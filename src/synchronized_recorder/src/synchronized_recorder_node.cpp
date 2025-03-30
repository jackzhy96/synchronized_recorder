#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <map>
#include <filesystem> // should be C++17
#include <getopt.h>


struct KinematicData {
    ros::Time stamp;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};



struct ImageData {
    ros::Time stamp;
    cv::Mat   image;
};


// Combine the data at an given timestamp into a packet
struct SyncedPacket {
    ros::Time        stamp;       // reference timestamp (from the matched data)
    ImageData        left_img;
    ImageData        right_img;
    KinematicData    kin_psm1;
    KinematicData    kin_psm2;
};


std::mutex              g_data_mutex;
std::condition_variable g_cv;
bool                    g_keep_running = true;


// buffers for left image, right image, and kinematics
std::queue<ImageData>    g_left_image_buffer;
std::queue<ImageData>    g_right_image_buffer;
std::queue<KinematicData> g_kinematic_buffer;
std::queue<KinematicData> g_kinematic_buffer_psm2;



// a queue for matched/synced data that needs to be written
std::queue<SyncedPacket> g_synced_queue;

const size_t MAX_IMAGE_BUFFER_SIZE     = 4000;
const size_t MAX_KINEMATIC_BUFFER_SIZE = 1000;
const size_t MAX_SYNCED_QUEUE_SIZE     = 100;      // queue for writing synced data

double TIME_TOL                = 0.005;    // 5ms tolerance
std::string camera_topic = "test";
std::string camera_type = "stereo";
std::vector<std::string> arms;
bool use_psm1 = false;
bool use_psm2 = false;
bool use_left_cam = false;
bool use_right_cam = false;


// created separate call back functions for left and right stereo camera
void imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg) {
    if (!use_left_cam) return;

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception (left): %s", e.what());
        return;
    }


    ImageData img_data;
    img_data.stamp = msg->header.stamp;
    img_data.image = cv_ptr->image.clone();


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_left_image_buffer.size() > MAX_IMAGE_BUFFER_SIZE) {
            g_left_image_buffer.pop();
        }
        g_left_image_buffer.push(img_data);
    }
}

// created separate call back functions for left and right stereo camera
void imageCallbackRight(const sensor_msgs::ImageConstPtr &msg) {
    if (!use_right_cam) return;

    cv_bridge::CvImageConstPtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception (right): %s", e.what());
        return;
    }


    ImageData img_data;
    img_data.stamp = msg->header.stamp;
    img_data.image = cv_ptr->image.clone();


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_right_image_buffer.size() > MAX_IMAGE_BUFFER_SIZE) {
            g_right_image_buffer.pop();
        }
        g_right_image_buffer.push(img_data);
    }
}


void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    if (!use_psm1) return;

    KinematicData kin_data;
    kin_data.stamp   = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer.size() > MAX_KINEMATIC_BUFFER_SIZE) {
            g_kinematic_buffer.pop();
        }

        g_kinematic_buffer.push(kin_data);
    }
}

// note: the function below is NOT a duplicate of the function above! The function below handles PSM2. 
void jointStateCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    if (!use_psm2) return;

    KinematicData kin_data;
    kin_data.stamp    = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer_psm2.size() > MAX_KINEMATIC_BUFFER_SIZE) {
            g_kinematic_buffer_psm2.pop();
        }

        g_kinematic_buffer_psm2.push(kin_data);
    }
}


// Thread #2: Synchronization
void syncThread() {

    std::cout << "------ syncThread started ------" << std::endl;

    while (ros::ok() && g_keep_running) {

        std::lock_guard<std::mutex> lock(g_data_mutex);


        if (g_synced_queue.size() >= MAX_SYNCED_QUEUE_SIZE) {
            // drop the oldest data from input buffers

            if (use_left_cam && !g_left_image_buffer.empty())    g_left_image_buffer.pop();
            if (use_right_cam && !g_right_image_buffer.empty())   g_right_image_buffer.pop();
            if (use_psm1 && !g_kinematic_buffer.empty())     g_kinematic_buffer.pop();
            if (use_psm2 && !g_kinematic_buffer_psm2.empty()) g_kinematic_buffer_psm2.pop();
            continue;
        }

        if ((use_left_cam && g_left_image_buffer.empty()) ||
            (use_right_cam && g_right_image_buffer.empty()) ||
            (use_psm1 && g_kinematic_buffer.empty()) ||
            (use_psm2 && g_kinematic_buffer_psm2.empty())) {
            continue;
        }

        ImageData     left_img;
        ImageData     right_img;
        if (use_left_cam)  left_img  = g_left_image_buffer.front();
        if (use_right_cam) right_img = g_right_image_buffer.front();
        KinematicData kin1;
        KinematicData kin2;

        if (use_psm1) kin1 = g_kinematic_buffer.front();
        if (use_psm2) kin2 = g_kinematic_buffer_psm2.front();

        ros::Time ref_stamp = use_psm1 ? kin1.stamp : (use_psm2 ? kin2.stamp : (use_left_cam ? left_img.stamp : right_img.stamp));
        double time_diff_left  = use_left_cam  ? fabs((left_img.stamp  - ref_stamp).toSec()) : 0.0;
        double time_diff_right = use_right_cam ? fabs((right_img.stamp - ref_stamp).toSec()) : 0.0;
        double time_diff_kin   = (use_psm1 && use_psm2) ? fabs((kin2.stamp - kin1.stamp).toSec()) : 0.0;

        if ((time_diff_left < TIME_TOL || !use_left_cam) &&
            (time_diff_right < TIME_TOL || !use_right_cam) &&
            (!use_psm1 || !use_psm2 || time_diff_kin < TIME_TOL)) 
        {
            SyncedPacket packet;
            packet.stamp     = ref_stamp; 
            if (use_left_cam)  packet.left_img  = left_img;
            if (use_right_cam) packet.right_img = right_img;
            if (use_psm1) packet.kin_psm1 = kin1;
            if (use_psm2) packet.kin_psm2 = kin2;

            g_synced_queue.push(packet);

            if (use_left_cam)  g_left_image_buffer.pop();
            if (use_right_cam) g_right_image_buffer.pop();
            if (use_psm1) g_kinematic_buffer.pop();
            if (use_psm2) g_kinematic_buffer_psm2.pop();

            g_cv.notify_one();
        } 
        else {
            ros::Time oldest_stamp = ref_stamp;
            if (use_left_cam && left_img.stamp < oldest_stamp) oldest_stamp = left_img.stamp;
            if (use_right_cam && right_img.stamp < oldest_stamp) oldest_stamp = right_img.stamp;
            if (use_psm1 && kin1.stamp < oldest_stamp) oldest_stamp = kin1.stamp;
            if (use_psm2 && kin2.stamp < oldest_stamp) oldest_stamp = kin2.stamp;

            if (use_left_cam && left_img.stamp == oldest_stamp) {
                g_left_image_buffer.pop();
            } 
            else if (use_right_cam && right_img.stamp == oldest_stamp) {
                g_right_image_buffer.pop();
            } 
            else if (use_psm1 && kin1.stamp == oldest_stamp) {
                g_kinematic_buffer.pop();
            } 
            else if (use_psm2 && kin2.stamp == oldest_stamp) {
                g_kinematic_buffer_psm2.pop();
            }
        }
    }
    std::cout << "------ syncThread exiting ------" << std::endl;
}




int main(int argc, char** argv) {
    std::cout << "------ program starting ------" << std::endl;

    int opt;
    while ((opt = getopt(argc, argv, "c:m:d:a:t:")) != -1) {
        switch (opt) {
            case 'c':
                camera_topic = std::string(optarg);
                break;
            case 'm':
                camera_type = std::string(optarg);
                break;
            case 'd':
                if (std::string(optarg) == "left") use_left_cam = true;
                else if (std::string(optarg) == "right") use_right_cam = true;
                break;
            case 'a':
                arms.emplace_back(optarg);
                break;
            case 't':
                TIME_TOL = std::stod(optarg);
                break;
            
            default:
                std::cerr << "Usage: rosrun synchronized_recorder synchronized_recorder_node -c <camera topic> -m <camera type; currently only support stereo> -d left -d right -a PSM1 -a PSM2 -t <time tolerance, in seconds>" << std::endl;
                std::cerr << "Example: rosrun synchronized_recorder synchronized_recorder_node -c test -m stereo -d left -a PSM2 -t 0.005" << std::endl;
                return 1;
        }
    }

    for (const auto& arm : arms) {
        if (arm == "PSM1") use_psm1 = true;
        if (arm == "PSM2") use_psm2 = true;
    }

    if (!use_left_cam && !use_right_cam) {
        std::cerr << "Error: No camera direction specified. Use -d left and/or -d right." << std::endl;
        return 1;
    }

    if (camera_type != "stereo") {
        std::cerr << "Error: Only stereo camera type is supported currently." << std::endl;
        return 1;
    }

    ros::init(argc, argv, "synchronized_recorder_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    if (use_left_cam)  image_transport::Subscriber left_sub  = it.subscribe("/" + camera_topic + "/left/image_raw",  1, imageCallbackLeft);
    if (use_right_cam) image_transport::Subscriber right_sub = it.subscribe("/" + camera_topic + "/right/image_raw", 1, imageCallbackRight);

    if (use_psm1) ros::Subscriber joint_sub_psm1 = nh.subscribe("/PSM1/measured_js", 1, jointStateCallback);
    if (use_psm2) ros::Subscriber joint_sub_psm2 = nh.subscribe("/PSM2/measured_js", 1, jointStateCallbackPSM2);

    std::cout << "------ start ROS spin in separate thread ------" << std::endl;
    std::thread ros_spin_thread([] {
        ros::spin();
        // Once spin() returns, shutdown
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    });

    std::thread sync_t(syncThread);

    ros_spin_thread.join();

    {
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    }

    sync_t.join();

    return 0;
}
