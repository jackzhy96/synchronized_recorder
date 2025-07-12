#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>   // this used for measured_cp
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/TwistStamped.h>   // cv
#include <cmath>                          

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
#include <string>
#include <vector>  // added for writer thread pool
#include <algorithm>   // for std::sort



void printUsage() {
    std::cout << "Usage: rosrun synchronized_recorder synchronized_recorder_node "
              << "-c <camera topic> -m <stereo|mono> [-d <left|right>] "
              << "-a PSM1 [-a PSM2] [-a ECM] -t <time_tolerance_seconds> [-v]  [-s]" << std::endl;
}

std::string g_camera_topic_base = "test";   // default camera topic base; still user must specify the topic in the argument
bool        g_use_left_image    = true;     // using left camera?
bool        g_use_right_image   = true;     // using right camera?
bool        g_record_psm1       = true;     // using PSM1 kinematics?
bool        g_record_psm2       = true;     // using PSM2 kinematics?
bool        g_record_ecm        = false;    // using ECM  kinematics?
bool        g_record_cv = false;            // measured_cv
bool        g_use_side_image = false;

double      g_time_tol          = 0.005;    // default is 5ms tolerance; still user must specify the tolerance in the argument


const int NUM_WRITER_THREADS = 4; // number of concurrent writer threads

// below helper function parses arguments and sets up the problem to do what the user requests
void parseArguments(int argc, char** argv) {
    bool camera_topic_set = false;
    bool camera_mode_set  = false;
    bool time_tol_set     = false;
    bool cv_set = false;
    bool side_set = false;

    std::vector<std::string> a_params;
    std::string camera_mode_str;
    std::string d_side;
    std::string kin_type_str;
    


    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-c") {
            if (i + 1 < argc) { g_camera_topic_base = argv[++i]; camera_topic_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-m") {
            if (i + 1 < argc) { camera_mode_str = argv[++i]; camera_mode_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-d") {
            if (i + 1 < argc) { d_side = argv[++i]; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-a") {
            if (i + 1 < argc) { a_params.emplace_back(argv[++i]); }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-t") {
            if (i + 1 < argc) { g_time_tol = std::stod(argv[++i]); time_tol_set = true; }
            else { printUsage(); exit(EXIT_FAILURE); }
        }
        else if (arg == "-v") {
            g_record_cv = true;
            cv_set = true;
        }
        else if (arg == "-s") {          
            g_use_side_image = true;
            side_set = true;
        }
        
        else {
            printUsage(); exit(EXIT_FAILURE);
        }
    }

    // give user warning that some required arg is missing
    // also give the correct arg format
    if (!camera_topic_set || !camera_mode_set || !time_tol_set ) {
        std::cerr << "Error: Missing required parameter(s)." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }

    // camera mode handling
    if (camera_mode_str == "stereo") {
        g_use_left_image  = true;
        g_use_right_image = true;
        if (!d_side.empty()) {
            std::cerr << "Error: -d should not be provided when -m is stereo." << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
    }
    else if (camera_mode_str == "mono") {
        if (d_side.empty()) {
            std::cerr << "Error: -d must be provided when -m is mono." << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
        if (d_side == "left") {
            g_use_left_image  = true;
            g_use_right_image = false;
        } else if (d_side == "right") {
            g_use_left_image  = false;
            g_use_right_image = true;
        } else {
            std::cerr << "Error: -d must be 'left' or 'right'." << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
    }
    else {
        std::cerr << "Error: invalid camera mode." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }

    // kinematic streams selection
    g_record_psm1 = false;
    g_record_psm2 = false;
    g_record_ecm  = false;

    if (a_params.empty()) {
        std::cerr << "Error: at least one -a parameter (PSM1, PSM2, or ECM) must be specified." << std::endl;
        printUsage();
        exit(EXIT_FAILURE);
    }

    // set up PSM1/PSM2/ECM recordings
    for (auto &a : a_params) {
        if (a == "PSM1")       g_record_psm1 = true;
        else if (a == "PSM2")  g_record_psm2 = true;
        else if (a == "ECM")   g_record_ecm  = true;
        else {
            std::cerr << "Error: invalid -a parameter: " << a << std::endl;
            printUsage();
            exit(EXIT_FAILURE);
        }
    }

}


struct KinematicData {
    ros::Time stamp;
    std::vector<double> position;
    std::vector<double> orientation;   // populated for cp
    std::vector<double> velocity;
    std::vector<double> effort;
    bool is_cp = false;                // true if data stream is coming from measured_cp
};



struct ImageData {
    ros::Time stamp;
    cv::Mat   image;
};


// combine the data at an given timestamp into a packet
struct SyncedPacket {
    ros::Time        stamp;       // reference timestamp (from the matched data)
    ImageData        left_img;
    ImageData        right_img;
    KinematicData    js_psm1, cp_psm1;
    KinematicData    js_psm2, cp_psm2;
    KinematicData    js_ecm , cp_ecm;
    KinematicData    sp_js_psm1, sp_cp_psm1;
    KinematicData    sp_js_psm2, sp_cp_psm2;
    KinematicData    sp_js_ecm , sp_cp_ecm;
    KinematicData    lcp_psm1;   // local measured_cp  
    KinematicData    lcp_psm2;   // local measured_cp
    KinematicData    lcp_ecm;    // local measured_cp
    KinematicData    jaw_meas_psm1;    // jaw streams
    KinematicData    jaw_set_psm1;
    KinematicData    jaw_meas_psm2;
    KinematicData    jaw_set_psm2;
    KinematicData    cv_psm1;
    KinematicData    cv_psm2;
    ImageData        side_img; 
    // note: jaw data is not available for ECM!
};


std::mutex              g_data_mutex;
std::condition_variable g_cv;
bool                    g_keep_running = true;


// buffers for left image, right image, and kinematics
std::queue<ImageData>     g_left_image_buffer;
std::queue<ImageData>     g_right_image_buffer;
std::queue<ImageData>     g_side_image_buffer;


std::queue<KinematicData> g_js_buffer_psm1, g_cp_buffer_psm1;
std::queue<KinematicData> g_js_buffer_psm2, g_cp_buffer_psm2;
std::queue<KinematicData> g_js_buffer_ecm , g_cp_buffer_ecm;

std::queue<KinematicData> g_lcp_buffer_psm1;
std::queue<KinematicData> g_lcp_buffer_psm2;
std::queue<KinematicData> g_lcp_buffer_ecm;

std::queue<KinematicData> g_cv_buffer_psm1;
std::queue<KinematicData> g_cv_buffer_psm2;

// latest setpoint snapshots

KinematicData g_setpoint_js_psm1, g_setpoint_cp_psm1;
KinematicData g_setpoint_js_psm2, g_setpoint_cp_psm2;
KinematicData g_setpoint_js_ecm , g_setpoint_cp_ecm ;



// latest jaw snapshots
KinematicData g_jaw_meas_psm1;
KinematicData g_jaw_set_psm1;
KinematicData g_jaw_meas_psm2;
KinematicData g_jaw_set_psm2;



// queue for matched / synced data that needs to be written
std::queue<SyncedPacket> g_synced_queue;

// feel free to experiment with different buffer sizes below; but note that at a certain point,
// this script's processing speed is capped by the hardware that you're running it on. 
const size_t MAX_BUFFER_SIZE         = 1000;     // 
const size_t MAX_SYNCED_QUEUE_SIZE   = 100;      // queue for writing synced data


// create separate call back function for left  stereo camera
void imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg) {

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
        if (g_left_image_buffer.size() > MAX_BUFFER_SIZE) {
            g_left_image_buffer.pop();
        }
        g_left_image_buffer.push(img_data);
    }


}

// create separate call back function for right stereo camera
void imageCallbackRight(const sensor_msgs::ImageConstPtr &msg) {

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
        if (g_right_image_buffer.size() > MAX_BUFFER_SIZE) {
            g_right_image_buffer.pop();
        }
        g_right_image_buffer.push(img_data);
    }


}
void imageCallbackSide(const sensor_msgs::ImageConstPtr& msg) {
    
    ImageData d;
    d.stamp  = msg->header.stamp;
    d.image  = cv_bridge::toCvShare(msg)->image.clone();
    std::lock_guard<std::mutex> lk(g_data_mutex);
    if (g_side_image_buffer.size() > MAX_BUFFER_SIZE) g_side_image_buffer.pop();
    g_side_image_buffer.push(d);
}

// get kinematic for PSM1
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp   = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;

    
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_js_buffer_psm1.size() > MAX_BUFFER_SIZE) {
            g_js_buffer_psm1.pop();
        }

        g_js_buffer_psm1.push(kin_data);
    }


}

// note: the function below is NOT a duplicate of the function above! The function below handles PSM2. 
void jointStateCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp    = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_js_buffer_psm2.size() > MAX_BUFFER_SIZE) {
            g_js_buffer_psm2.pop();
        }
        
        g_js_buffer_psm2.push(kin_data);
    }


}

// note: the function below handles ECM.
void jointStateCallbackECM(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp    = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;


    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_js_buffer_ecm.size() > MAX_BUFFER_SIZE) {
            g_js_buffer_ecm.pop();
        }
        
        g_js_buffer_ecm.push(kin_data);
    }
}



// ------------------- below functions are callbacks for measured_cp -------------------

void poseCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = {msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z};
    kin_data.orientation = {msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w};
    kin_data.is_cp = true;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_cp_buffer_psm1.size() > MAX_BUFFER_SIZE) {
            g_cp_buffer_psm1.pop();
        }
        g_cp_buffer_psm1.push(kin_data);
    }
}

void poseCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = {msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z};
    kin_data.orientation = {msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w};
    kin_data.is_cp = true;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_cp_buffer_psm2.size() > MAX_BUFFER_SIZE) {
            g_cp_buffer_psm2.pop();
        }
        g_cp_buffer_psm2.push(kin_data);
    }
}

void poseCallbackECM(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = {msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z};
    kin_data.orientation = {msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z,
                            msg->pose.orientation.w};
    kin_data.is_cp = true;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_cp_buffer_ecm.size() > MAX_BUFFER_SIZE) {
            g_cp_buffer_ecm.pop();
        }
        g_cp_buffer_ecm.push(kin_data);
    }
}


void localCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    KinematicData kd;
    kd.stamp      = msg->header.stamp;
    kd.position   = {msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z};
    kd.orientation= {msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w};
    kd.is_cp = true;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    if (g_lcp_buffer_psm1.size() > MAX_BUFFER_SIZE) g_lcp_buffer_psm1.pop();
    g_lcp_buffer_psm1.push(kd);
}
void localCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    KinematicData kd;
    kd.stamp      = msg->header.stamp;
    kd.position   = {msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z};
    kd.orientation= {msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w};
    kd.is_cp = true;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    if (g_lcp_buffer_psm2.size() > MAX_BUFFER_SIZE) g_lcp_buffer_psm2.pop();
    g_lcp_buffer_psm2.push(kd);
}
void localCPCallbackECM(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    KinematicData kd;
    kd.stamp      = msg->header.stamp;
    kd.position   = {msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z};
    kd.orientation= {msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w};
    kd.is_cp = true;

    std::lock_guard<std::mutex> lk(g_data_mutex);
    if (g_lcp_buffer_ecm.size() > MAX_BUFFER_SIZE) g_lcp_buffer_ecm.pop();
    g_lcp_buffer_ecm.push(kd);
}


void cvCallbackPSM2(const geometry_msgs::TwistStamped::ConstPtr &msg){
    KinematicData kd;
    kd.stamp = msg->header.stamp;
    kd.velocity = { msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z };
    kd.is_cp = false;   // reuse 标志
    std::lock_guard<std::mutex> lk(g_data_mutex);
    if (g_cv_buffer_psm2.size() > MAX_BUFFER_SIZE) g_cv_buffer_psm2.pop();
    g_cv_buffer_psm2.push(kd);
}
void cvCallbackPSM1(const geometry_msgs::TwistStamped::ConstPtr &msg){
    KinematicData kd;
    kd.stamp = msg->header.stamp;
    kd.velocity = { msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                    msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z };
    kd.is_cp = false;   // reuse 标志
    std::lock_guard<std::mutex> lk(g_data_mutex);
    if (g_cv_buffer_psm1.size() > MAX_BUFFER_SIZE) g_cv_buffer_psm1.pop();
    g_cv_buffer_psm1.push(kd);
}

// ------------------- below functions are callbacks for setpoint streams -------------------

void setpointJSCallbackPSM1(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_js_psm1.stamp     = msg->header.stamp;
    g_setpoint_js_psm1.position  = msg->position;
    g_setpoint_js_psm1.velocity  = msg->velocity;
    g_setpoint_js_psm1.effort    = msg->effort;
    g_setpoint_js_psm1.orientation.clear();
    g_setpoint_js_psm1.is_cp = false;
}

void setpointJSCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_js_psm2.stamp     = msg->header.stamp;
    g_setpoint_js_psm2.position  = msg->position;
    g_setpoint_js_psm2.velocity  = msg->velocity;
    g_setpoint_js_psm2.effort    = msg->effort;
    g_setpoint_js_psm2.orientation.clear();
    g_setpoint_js_psm2.is_cp = false;
}

void setpointJSCallbackECM(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_js_ecm.stamp     = msg->header.stamp;
    g_setpoint_js_ecm.position  = msg->position;
    g_setpoint_js_ecm.velocity  = msg->velocity;
    g_setpoint_js_ecm.effort    = msg->effort;
    g_setpoint_js_ecm.orientation.clear();
    g_setpoint_js_ecm.is_cp = false;
}

void setpointCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_cp_psm1.stamp = msg->header.stamp;
    g_setpoint_cp_psm1.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_setpoint_cp_psm1.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_setpoint_cp_psm1.velocity.clear();
    g_setpoint_cp_psm1.effort.clear();
    g_setpoint_cp_psm1.is_cp = true;
}

void setpointCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_cp_psm2.stamp = msg->header.stamp;
    g_setpoint_cp_psm2.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_setpoint_cp_psm2.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_setpoint_cp_psm2.velocity.clear();
    g_setpoint_cp_psm2.effort.clear();
    g_setpoint_cp_psm2.is_cp = true;
}

void setpointCPCallbackECM(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_setpoint_cp_ecm.stamp = msg->header.stamp;
    g_setpoint_cp_ecm.position = {msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z};
    g_setpoint_cp_ecm.orientation = {msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z,
                                  msg->pose.orientation.w};
    g_setpoint_cp_ecm.velocity.clear();
    g_setpoint_cp_ecm.effort.clear();
    g_setpoint_cp_ecm.is_cp = true;
}



// ------------------- below functions are callbacks for jaw streams -------------------
// note that ECM doesn't carry jaw data

void jawMeasuredJSCallbackPSM1(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm1.stamp    = msg->header.stamp;
    g_jaw_meas_psm1.position = msg->position;
    g_jaw_meas_psm1.velocity = msg->velocity;
    g_jaw_meas_psm1.effort   = msg->effort;
    g_jaw_meas_psm1.orientation.clear();
    g_jaw_meas_psm1.is_cp = false;
}

void jawMeasuredJSCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm2.stamp    = msg->header.stamp;
    g_jaw_meas_psm2.position = msg->position;
    g_jaw_meas_psm2.velocity = msg->velocity;
    g_jaw_meas_psm2.effort   = msg->effort;
    g_jaw_meas_psm2.orientation.clear();
    g_jaw_meas_psm2.is_cp = false;
}

void jawSetpointJSCallbackPSM1(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm1.stamp    = msg->header.stamp;
    g_jaw_set_psm1.position = msg->position;
    g_jaw_set_psm1.velocity = msg->velocity;
    g_jaw_set_psm1.effort   = msg->effort;
    g_jaw_set_psm1.orientation.clear();
    g_jaw_set_psm1.is_cp = false;
}

void jawSetpointJSCallbackPSM2(const sensor_msgs::JointState::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm2.stamp    = msg->header.stamp;
    g_jaw_set_psm2.position = msg->position;
    g_jaw_set_psm2.velocity = msg->velocity;
    g_jaw_set_psm2.effort   = msg->effort;
    g_jaw_set_psm2.orientation.clear();
    g_jaw_set_psm2.is_cp = false;
}

void jawMeasuredCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm1.stamp = msg->header.stamp;
    g_jaw_meas_psm1.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_jaw_meas_psm1.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_jaw_meas_psm1.velocity.clear();
    g_jaw_meas_psm1.effort.clear();
    g_jaw_meas_psm1.is_cp = true;
}

void jawMeasuredCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_meas_psm2.stamp = msg->header.stamp;
    g_jaw_meas_psm2.position = {msg->pose.position.x,
                                msg->pose.position.y,
                                msg->pose.position.z};
    g_jaw_meas_psm2.orientation = {msg->pose.orientation.x,
                                   msg->pose.orientation.y,
                                   msg->pose.orientation.z,
                                   msg->pose.orientation.w};
    g_jaw_meas_psm2.velocity.clear();
    g_jaw_meas_psm2.effort.clear();
    g_jaw_meas_psm2.is_cp = true;
}

void jawSetpointCPCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm1.stamp = msg->header.stamp;
    g_jaw_set_psm1.position = {msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z};
    g_jaw_set_psm1.orientation = {msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z,
                                  msg->pose.orientation.w};
    g_jaw_set_psm1.velocity.clear();
    g_jaw_set_psm1.effort.clear();
    g_jaw_set_psm1.is_cp = true;
}

void jawSetpointCPCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_jaw_set_psm2.stamp = msg->header.stamp;
    g_jaw_set_psm2.position = {msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z};
    g_jaw_set_psm2.orientation = {msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z,
                                  msg->pose.orientation.w};
    g_jaw_set_psm2.velocity.clear();
    g_jaw_set_psm2.effort.clear();
    g_jaw_set_psm2.is_cp = true;
}



// Thread #2: Synchronization
void syncThread() {

    std::cout << "------ syncThread started ------" << std::endl;
    
    while (ros::ok() && g_keep_running) {

        std::lock_guard<std::mutex> lock(g_data_mutex);


        if (g_synced_queue.size() >= MAX_SYNCED_QUEUE_SIZE)
        {
            if (g_use_left_image  && !g_left_image_buffer.empty())  g_left_image_buffer.pop();
            if (g_use_right_image && !g_right_image_buffer.empty()) g_right_image_buffer.pop();
            if (g_use_side_image  && !g_side_image_buffer.empty())  g_side_image_buffer.pop();

            if (g_record_psm1)
            {
                if (!g_js_buffer_psm1.empty()) g_js_buffer_psm1.pop();
                if (!g_cp_buffer_psm1.empty()) g_cp_buffer_psm1.pop();
            }
            if (g_record_psm2)
            {
                if (!g_js_buffer_psm2.empty()) g_js_buffer_psm2.pop();
                if (!g_cp_buffer_psm2.empty()) g_cp_buffer_psm2.pop();
            }
            if (g_record_ecm)
            {
                if (!g_js_buffer_ecm.empty()) g_js_buffer_ecm.pop();
                if (!g_cp_buffer_ecm.empty()) g_cp_buffer_ecm.pop();
            }
            continue;
        }

        // if any required buffer is empty, can't sync
        bool ready =
            (!g_use_left_image  || !g_left_image_buffer.empty())  &&
            (!g_use_right_image || !g_right_image_buffer.empty()) &&
            (!g_use_side_image  || !g_side_image_buffer.empty())  &&

            (!g_record_psm1 || 
                (!g_js_buffer_psm1.empty() && !g_cp_buffer_psm1.empty()) && !g_lcp_buffer_psm1.empty())&&
            (!g_record_psm2 || 
                (!g_js_buffer_psm2.empty() && !g_cp_buffer_psm2.empty()) && !g_lcp_buffer_psm2.empty())&&
            (!g_record_ecm  || 
                (!g_js_buffer_ecm.empty() && !g_cp_buffer_ecm.empty()) && !g_lcp_buffer_ecm.empty())&&

            (!g_record_cv || !g_record_psm1 || !g_cv_buffer_psm1.empty()) &&
            (!g_record_cv || !g_record_psm2 || !g_cv_buffer_psm2.empty());

        if (!ready)  continue;

        
        ImageData     left_img;
        ImageData     right_img;
        KinematicData js1, cp1, js2, cp2, js3, cp3;
        KinematicData lcp1, lcp2, lcp3;
        KinematicData cv1, cv2;
        ImageData     side_img;

        if (g_use_left_image)   left_img  = g_left_image_buffer.front();
        if (g_use_right_image)  right_img = g_right_image_buffer.front();
        if (g_use_side_image)   side_img  = g_side_image_buffer.front();

        if (g_record_psm1)
        {
            js1 = g_js_buffer_psm1.front();
            cp1 = g_cp_buffer_psm1.front();
            lcp1 = g_lcp_buffer_psm1.front();
        }
        if (g_record_psm2)
        {
            js2 = g_js_buffer_psm2.front();
            cp2 = g_cp_buffer_psm2.front();
            lcp2 = g_lcp_buffer_psm2.front();
        }
        if (g_record_ecm)
        {
            js3 = g_js_buffer_ecm.front();
            cp3 = g_cp_buffer_ecm.front();
            lcp3 = g_lcp_buffer_ecm.front();
        }

        if (g_record_cv && g_record_psm1) cv1 = g_cv_buffer_psm1.front();
        if (g_record_cv && g_record_psm2) cv2 = g_cv_buffer_psm2.front();


        ros::Time ref_stamp;
        if (g_record_psm1)            ref_stamp = js1.stamp;
        else if (g_record_psm2)       ref_stamp = js2.stamp;
        else                           ref_stamp = js3.stamp;

        bool in_tol = true;

        if (g_use_left_image)  in_tol &= fabs((left_img.stamp  - ref_stamp).toSec())  < g_time_tol;
        if (g_use_right_image) in_tol &= fabs((right_img.stamp - ref_stamp).toSec())  < g_time_tol;
        if (g_record_cv && g_record_psm1) in_tol &= fabs((cv1.stamp - ref_stamp).toSec()) < g_time_tol;
        if (g_record_cv && g_record_psm2) in_tol &= fabs((cv2.stamp - ref_stamp).toSec()) < g_time_tol;
        if (g_use_side_image)  in_tol &= fabs((side_img.stamp  - ref_stamp).toSec())  < g_time_tol;


        // Lines below are commented out, because we don't need to compare kinematic data to each other
        // in terms of time discrepency. each set of kinematic data is already compared to the images timestamp.

        // My experiments have shown that de-commenting the lines below do very little to reducing time discrepency.

        // if (g_record_psm1 && g_record_psm2) in_tol &= fabs((kin2.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (g_record_psm1 && g_record_ecm)  in_tol &= fabs((kin3.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (!g_record_psm1 && g_record_psm2 && g_record_ecm) in_tol &= fabs((kin3.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (g_record_psm1) in_tol &= fabs((lcp1.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (g_record_psm2) in_tol &= fabs((lcp2.stamp - ref_stamp).toSec()) < g_time_tol;
        // if (g_record_ecm)  in_tol &= fabs((lcp3.stamp - ref_stamp).toSec()) < g_time_tol;


        if (in_tol) 
        {
            SyncedPacket packet;

            packet.stamp     = ref_stamp; 
            packet.left_img  = left_img;
            packet.right_img = right_img;
            packet.side_img =  side_img;

            packet.js_psm1 = js1;   packet.cp_psm1 = cp1;
            packet.js_psm2 = js2;   packet.cp_psm2 = cp2;
            packet.js_ecm  = js3;   packet.cp_ecm  = cp3;

            packet.sp_js_psm1 = g_setpoint_js_psm1;
            packet.sp_cp_psm1 = g_setpoint_cp_psm1;

            packet.sp_js_psm2 = g_setpoint_js_psm2;
            packet.sp_cp_psm2 = g_setpoint_cp_psm2;

            packet.sp_js_ecm  = g_setpoint_js_ecm;
            packet.sp_cp_ecm  = g_setpoint_cp_ecm;

            packet.lcp_psm1 = lcp1;      
            packet.lcp_psm2 = lcp2;
            packet.lcp_ecm  = lcp3;

            packet.jaw_meas_psm1 = g_jaw_meas_psm1;
            packet.jaw_set_psm1  = g_jaw_set_psm1;
            packet.jaw_meas_psm2 = g_jaw_meas_psm2;
            packet.jaw_set_psm2  = g_jaw_set_psm2;
            if (g_record_cv) {
                packet.cv_psm1 = cv1;
                packet.cv_psm2 = cv2;
  
            }
            
            g_synced_queue.push(packet);

            // pop the buffers we used
            if (g_use_left_image)   g_left_image_buffer.pop();
            if (g_use_right_image)  g_right_image_buffer.pop();
            if (g_use_side_image)   g_side_image_buffer.pop();

            if (g_record_psm1)
            { g_js_buffer_psm1.pop(); g_cp_buffer_psm1.pop(); g_lcp_buffer_psm1.pop();}
            if (g_record_psm2)
            { g_js_buffer_psm2.pop(); g_cp_buffer_psm2.pop(); g_lcp_buffer_psm2.pop();}
            if (g_record_ecm)
            { g_js_buffer_ecm.pop();  g_cp_buffer_ecm.pop();  g_lcp_buffer_ecm.pop();}

            if (g_record_cv && g_record_psm1) g_cv_buffer_psm1.pop();
            if (g_record_cv && g_record_psm2) g_cv_buffer_psm2.pop();

            g_cv.notify_one();
        } 
        else {
            ros::Time oldest_stamp;
            bool init = false;
            auto consider_stamp = [&](ros::Time t){ if (!init || t < oldest_stamp){ oldest_stamp = t; init = true; } };

            if (g_use_left_image)  consider_stamp(left_img.stamp);
            if (g_use_side_image)  consider_stamp(side_img.stamp);
            if (g_use_right_image) consider_stamp(right_img.stamp);

            if (g_record_psm1){ consider_stamp(js1.stamp);  consider_stamp(cp1.stamp); consider_stamp(lcp1.stamp);}
            if (g_record_psm2){ consider_stamp(js2.stamp);  consider_stamp(cp2.stamp); consider_stamp(lcp2.stamp);}
            if (g_record_ecm ){ consider_stamp(js3.stamp);  consider_stamp(cp3.stamp); consider_stamp(lcp3.stamp);}

            if (g_record_cv && g_record_psm1) consider_stamp(cv1.stamp);
            if (g_record_cv && g_record_psm2) consider_stamp(cv2.stamp);

            if (g_use_left_image  && left_img.stamp  == oldest_stamp) g_left_image_buffer.pop();
            else if (g_use_right_image && right_img.stamp == oldest_stamp) g_right_image_buffer.pop();
            else if (g_use_side_image  && side_img.stamp == oldest_stamp)  g_side_image_buffer.pop();

            else if (g_record_psm1 && js1.stamp == oldest_stamp) g_js_buffer_psm1.pop();
            else if (g_record_psm1 && cp1.stamp == oldest_stamp) g_cp_buffer_psm1.pop();

            else if (g_record_psm2 && js2.stamp == oldest_stamp) g_js_buffer_psm2.pop();
            else if (g_record_psm2 && cp2.stamp == oldest_stamp) g_cp_buffer_psm2.pop();

            else if (g_record_ecm  && js3.stamp == oldest_stamp) g_js_buffer_ecm.pop();
            else if (g_record_ecm  && cp3.stamp == oldest_stamp) g_cp_buffer_ecm.pop();

            else if (g_record_cv && g_record_psm1 && cv1.stamp == oldest_stamp) g_cv_buffer_psm1.pop();
            else if (g_record_cv && g_record_psm2 && cv2.stamp == oldest_stamp) g_cv_buffer_psm2.pop();

            else if (g_record_psm1 && lcp1.stamp == oldest_stamp) g_lcp_buffer_psm1.pop();
            else if (g_record_psm2 && lcp2.stamp == oldest_stamp) g_lcp_buffer_psm2.pop();
            else if (g_record_ecm  && lcp3.stamp == oldest_stamp) g_lcp_buffer_ecm.pop();

        }

    }
    std::cout << "------ syncThread exiting ------" << std::endl;
}




// Thread #3: Image Writer: combined writing -see packet set-up above
void writerThread() {
    std::cout << "------ writerThread started ------" << std::endl;

    // fast PNG params: compression level 1 (0-9, lower is faster)
    static const std::vector<int> PNG_FAST_PARAMS = {cv::IMWRITE_PNG_COMPRESSION, 1};

    while (ros::ok() && g_keep_running) {
        std::unique_lock<std::mutex> lock(g_data_mutex);
        g_cv.wait(lock, [] {
            return !g_synced_queue.empty() || !g_keep_running;
        });


        if (!g_keep_running && g_synced_queue.empty()) {
            break;
        }


        SyncedPacket packet = g_synced_queue.front();
        g_synced_queue.pop();
        lock.unlock();


        ros::WallTime wall_now = ros::WallTime::now();
        std::ostringstream folder_name;
        folder_name << "recorded_data/"
                    << wall_now.sec << "_"
                    << wall_now.nsec;
        std::string final_folder = folder_name.str();


        std::error_code ec;
        std::filesystem::create_directories(final_folder, ec);


        if (ec) {
            std::cerr << "Failed to create directory " << final_folder 
                      << ": " << ec.message() << std::endl;
            continue;
        }



        // Convert from BGR -> RGB as requested
        cv::Mat left_rgb, right_rgb, side_rgb;

        bool ok_left  = true;
        bool ok_right = true;
        bool ok_side = true;

        if (g_use_left_image) {
            cv::cvtColor(packet.left_img.image,  left_rgb,  cv::COLOR_BGR2RGB);
            std::string left_img_path  = final_folder + "/image_left.png";
            ok_left  = cv::imwrite(left_img_path,  left_rgb, PNG_FAST_PARAMS);
        }

        if (g_use_right_image) {
            cv::cvtColor(packet.right_img.image, right_rgb, cv::COLOR_BGR2RGB);
            std::string right_img_path = final_folder + "/image_right.png";
            ok_right = cv::imwrite(right_img_path, right_rgb, PNG_FAST_PARAMS);
        }

        if (g_use_side_image) {
            cv::cvtColor(packet.side_img.image, side_rgb, cv::COLOR_BGR2RGB);
            std::string side_path = final_folder + "/image_side.png";
            ok_side = cv::imwrite(side_path, side_rgb, PNG_FAST_PARAMS);
        }

        if ((g_use_side_image && !ok_side) ||(g_use_left_image && !ok_left) || (g_use_right_image && !ok_right)) {
            std::cerr << "Failed to write images in " << final_folder << std::endl;
            continue;
        }



        // --- Write PSM1 kinematics ---
        if (g_record_psm1) {
            std::string kin_path_psm1 = final_folder + "/kinematics_PSM1.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.cp_psm1.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.cp_psm1.stamp.nsec;

                /* ---------------- measured_js ---------------- */
            Json::Value meas_js;
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.js_psm1.position) pos.append(p);
                meas_js["position"] = pos;

                Json::Value vel(Json::arrayValue);
                for (double v : packet.js_psm1.velocity) vel.append(v);
                meas_js["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (double e : packet.js_psm1.effort)   eff.append(e);
                meas_js["effort"] = eff;
            }

            /* ---------------- measured_cp ---------------- */
            Json::Value meas_cp;
            if (!packet.cp_psm1.position.empty())
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.cp_psm1.position) pos.append(p);
                meas_cp["position"] = pos;
            }
            if (!packet.cp_psm1.orientation.empty())
            {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.cp_psm1.orientation) ori.append(o);
                meas_cp["orientation"] = ori;
            }
            if (g_record_cv && !packet.cv_psm1.velocity.empty())
            {
                Json::Value cart_vel(Json::arrayValue);
                for (double v : packet.cv_psm1.velocity) cart_vel.append(v);
                meas_cp["velocity"] = cart_vel;
            }

            Json::Value measured_block;
            measured_block["js"] = meas_js;
            measured_block["cp"] = meas_cp;


            Json::Value set_js, set_cp;
            /* --- set_js（joint space）--- */
            if (!packet.sp_js_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.sp_js_psm1.position) pos.append(p);
                set_js["position"] = pos;

                Json::Value vel(Json::arrayValue);
                for (double v : packet.sp_js_psm1.velocity) vel.append(v);
                set_js["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (double e : packet.sp_js_psm1.effort)  eff.append(e);
                set_js["effort"] = eff;
            }

            /* --- set_cp（cartesian space）--- */
            if (!packet.sp_cp_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.sp_cp_psm1.position) pos.append(p);
                set_cp["position"] = pos;
            }
            if (!packet.sp_cp_psm1.orientation.empty()) {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.sp_cp_psm1.orientation) ori.append(o);
                set_cp["orientation"] = ori;
            }

            Json::Value set_block;
            set_block["js"] = set_js;
            set_block["cp"] = set_cp;

            /* -------- local_cp -------- */
            Json::Value local_cp;
            if (!packet.lcp_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.lcp_psm1.position) pos.append(p);
                local_cp["position"] = pos;
            }
            if (!packet.lcp_psm1.orientation.empty()) {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.lcp_psm1.orientation) ori.append(o);
                local_cp["orientation"] = ori;
}


            Json::Value jaw_meas;
            if (!packet.jaw_meas_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_meas_psm1.position) { pos.append(p); }
                jaw_meas["position"] = pos;
            }

            Json::Value jaw_set;
            if (!packet.jaw_set_psm1.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_set_psm1.position) { pos.append(p); }
                jaw_set["position"] = pos;
            }

            Json::Value arm_block;
            arm_block["measured_data"] = measured_block;
            arm_block["setpoint_data"] = set_block;
            arm_block["local_cp"]      = local_cp; 

            Json::Value jaw_block;
            jaw_block["measured_data"] = jaw_meas;
            jaw_block["setpoint_data"] = jaw_set;

            root["arm"] = arm_block;
            root["jaw"] = jaw_block;

            std::ofstream file(kin_path_psm1);
            if (!file) {
                std::cerr << "Failed to open kinematics_PSM1.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }

        
        // --- Write PSM2 kinematics ---
        if (g_record_psm2) {
            std::string kin_path_psm2 = final_folder + "/kinematics_PSM2.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.cp_psm2.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.cp_psm2.stamp.nsec;

            Json::Value meas_js;
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.js_psm2.position) pos.append(p);
                meas_js["position"] = pos;
        
                Json::Value vel(Json::arrayValue);
                for (double v : packet.js_psm2.velocity) vel.append(v);
                meas_js["velocity"] = vel;
        
                Json::Value eff(Json::arrayValue);
                for (double e : packet.js_psm2.effort)   eff.append(e);
                meas_js["effort"] = eff;
            }
        
            /* ---------------- measured_cp ---------------- */
            Json::Value meas_cp;
            if (!packet.cp_psm2.position.empty())
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.cp_psm2.position) pos.append(p);
                meas_cp["position"] = pos;
            }
            if (!packet.cp_psm2.orientation.empty())
            {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.cp_psm2.orientation) ori.append(o);
                meas_cp["orientation"] = ori;
            }
            if (g_record_cv && !packet.cv_psm2.velocity.empty())
            {
                Json::Value cart_vel(Json::arrayValue);
                for (double v : packet.cv_psm2.velocity) cart_vel.append(v);
                meas_cp["velocity"] = cart_vel;
            }

            Json::Value measured_block;
            measured_block["js"] = meas_js;
            measured_block["cp"] = meas_cp;

            Json::Value set_js, set_cp;
            /* --- set_js（joint space）--- */
            if (!packet.sp_js_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.sp_js_psm2.position) pos.append(p);
                set_js["position"] = pos;

                Json::Value vel(Json::arrayValue);
                for (double v : packet.sp_js_psm2.velocity) vel.append(v);
                set_js["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (double e : packet.sp_js_psm2.effort)  eff.append(e);
                set_js["effort"] = eff;
            }

            /* --- set_cp（cartesian space）--- */
            if (!packet.sp_cp_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.sp_cp_psm2.position) pos.append(p);
                set_cp["position"] = pos;
            }
            if (!packet.sp_cp_psm2.orientation.empty()) {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.sp_cp_psm2.orientation) ori.append(o);
                set_cp["orientation"] = ori;
            }

            Json::Value set_block;
            set_block["js"] = set_js;
            set_block["cp"] = set_cp;

            /* -------- local_cp -------- */
            Json::Value local_cp;
            if (!packet.lcp_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.lcp_psm2.position) pos.append(p);
                local_cp["position"] = pos;
            }
            if (!packet.lcp_psm2.orientation.empty()) {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.lcp_psm2.orientation) ori.append(o);
                local_cp["orientation"] = ori;
            }


            Json::Value jaw_meas;
            if (!packet.jaw_meas_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_meas_psm2.position) { pos.append(p); }
                jaw_meas["position"] = pos;
            }

            Json::Value jaw_set;
            if (!packet.jaw_set_psm2.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (auto &p : packet.jaw_set_psm2.position) { pos.append(p); }
                jaw_set["position"] = pos;
            }

            Json::Value arm_block;
            arm_block["measured_data"] = measured_block;
            arm_block["setpoint_data"] = set_block;
            arm_block["local_cp"]      = local_cp; 

            Json::Value jaw_block;
            jaw_block["measured_data"] = jaw_meas;
            jaw_block["setpoint_data"] = jaw_set;

            root["arm"] = arm_block;
            root["jaw"] = jaw_block;

            std::ofstream file(kin_path_psm2);
            if (!file) {
                std::cerr << "Failed to open kinematics_PSM2.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }

        
        // --- Write ECM kinematics ---
        if (g_record_ecm) {
            std::string kin_path_ecm = final_folder + "/kinematics_ECM.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.cp_ecm.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.cp_ecm.stamp.nsec;

                    /* ---------- measured_js ---------- */
            Json::Value meas_js;
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.js_ecm.position)  pos.append(p);
                meas_js["position"] = pos;

                Json::Value vel(Json::arrayValue);
                for (double v : packet.js_ecm.velocity)  vel.append(v);
                meas_js["velocity"] = vel;

                Json::Value eff(Json::arrayValue);
                for (double e : packet.js_ecm.effort)    eff.append(e);
                meas_js["effort"] = eff;
            }

            /* ---------- measured_cp ---------- */
            Json::Value meas_cp;
            if (!packet.cp_ecm.position.empty())
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.cp_ecm.position)  pos.append(p);
                meas_cp["position"] = pos;
            }
            if (!packet.cp_ecm.orientation.empty())
            {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.cp_ecm.orientation) ori.append(o);
                meas_cp["orientation"] = ori;
            }

            /* ---------- measured_data block ---------- */
            Json::Value measured_block;
            measured_block["js"] = meas_js;
            measured_block["cp"] = meas_cp;

            /* ---------- set-point (JS / CP) ---------- */
            Json::Value set_js, set_cp;

            /* JS (set_js) */
            if (!packet.sp_js_ecm.position.empty())
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.sp_js_ecm.position) pos.append(p);
                set_js["position"] = pos;
            }
            if (!packet.sp_js_ecm.velocity.empty())
            {
                Json::Value vel(Json::arrayValue);
                for (double v : packet.sp_js_ecm.velocity) vel.append(v);
                set_js["velocity"] = vel;
            }
            if (!packet.sp_js_ecm.effort.empty())
            {
                Json::Value eff(Json::arrayValue);
                for (double e : packet.sp_js_ecm.effort) eff.append(e);
                set_js["effort"] = eff;
            }

            /* CP (set_cp) */
            if (!packet.sp_cp_ecm.position.empty())
            {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.sp_cp_ecm.position) pos.append(p);
                set_cp["position"] = pos;
            }
            if (!packet.sp_cp_ecm.orientation.empty())
            {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.sp_cp_ecm.orientation) ori.append(o);
                set_cp["orientation"] = ori;
            }

            Json::Value set_block;
            set_block["js"] = set_js;
            set_block["cp"] = set_cp;

            /* -------- local_cp -------- */
            Json::Value local_cp;
            if (!packet.lcp_ecm.position.empty()) {
                Json::Value pos(Json::arrayValue);
                for (double p : packet.lcp_ecm.position) pos.append(p);
                local_cp["position"] = pos;
            }
            if (!packet.lcp_ecm.orientation.empty()) {
                Json::Value ori(Json::arrayValue);
                for (double o : packet.lcp_ecm.orientation) ori.append(o);
                local_cp["orientation"] = ori;
            }


            /* -------- arm_block -------- */
            Json::Value arm_block;
            arm_block["measured_data"] = measured_block;
            arm_block["setpoint_data"] = set_block; 
            arm_block["local_cp"]      = local_cp; 

            root["arm"] = arm_block; 

            std::ofstream file(kin_path_ecm);
            if (!file) {
                std::cerr << "Failed to open kinematics_ECM.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }


        std::cout << "[writerThread] Wrote data to " 
                  << final_folder << std::endl;
    }


    std::cout << "------ writerThread exiting ------" << std::endl;
}




// ---------------------------- post-processing functions below ----------------------------

// remove incomplete folders - these folders are not useful because either images or kinematic data sets are missing
void cleanupFolders() {
    std::cout << "------ cleanup step: checking for incomplete folders ------" << std::endl;
    std::filesystem::path base_dir("recorded_data");

    if (!std::filesystem::exists(base_dir)) {
        return;
    }

    for (auto &entry : std::filesystem::directory_iterator(base_dir)) {

        if (entry.is_directory()) {

            auto img_left_path  = entry.path() / "image_left.png";
            auto img_right_path = entry.path() / "image_right.png";
            auto img_side_path = entry.path() / "image_side.png";
            auto json_psm1_path = entry.path() / "kinematics_PSM1.json";
            auto json_psm2_path = entry.path() / "kinematics_PSM2.json";
            auto json_ecm_path  = entry.path() / "kinematics_ECM.json";

            
            bool left_exists   = std::filesystem::exists(img_left_path);
            bool right_exists  = std::filesystem::exists(img_right_path);
            bool side_exists   = std::filesystem::exists(img_side_path);
            bool psm1_exists   = std::filesystem::exists(json_psm1_path);
            bool psm2_exists   = std::filesystem::exists(json_psm2_path);
            bool ecm_exists    = std::filesystem::exists(json_ecm_path);

            bool remove = false;
            if (g_use_left_image  && !left_exists)  remove = true;
            if (g_use_right_image && !right_exists) remove = true;
            if (g_use_side_image && !side_exists)   remove = true;
            if (g_record_psm1     && !psm1_exists)  remove = true;
            if (g_record_psm2     && !psm2_exists)  remove = true;
            if (g_record_ecm      && !ecm_exists)   remove = true;


            // If missing any required file, remove the folder
            if (remove) {
                std::cout << "Removing incomplete folder: " 
                          << entry.path().string() << std::endl;
                std::filesystem::remove_all(entry.path());
            }
        }
    }
}


// testing tool to see the frequency of the recording (counting complete recording sets only)
void countFoldersPerSecond() {
    std::cout << " ------ Gathering output frequency during runtime ------" << std::endl;
    std::filesystem::path base_dir("recorded_data");
    if (!std::filesystem::exists(base_dir)) {
        std::cout << "Error. Output directory not found." << std::endl;
        return;
    }

    std::map<std::string, int> folder_count;


    for (const auto &entry : std::filesystem::directory_iterator(base_dir)) {
        if (entry.is_directory()) {
            std::string folder_name = entry.path().filename().string();

            // count how many outputs occur at a given second
            size_t underscore_pos = folder_name.find('_');
            if (underscore_pos != std::string::npos) {
                std::string sec_part = folder_name.substr(0, underscore_pos);
                folder_count[sec_part]++;
            }
        }
    }

    // Print output freq
    for (const auto &[sec, count] : folder_count) {
        std::cout << "At " << sec << "second: " << count << " Hz" << std::endl;
    }
}




void reformatDataStorage() {
    std::string base_folder = "data_save_folder";
    std::filesystem::create_directories(base_folder + "/image/left");
    std::filesystem::create_directories(base_folder + "/image/right");
    std::filesystem::create_directories(base_folder + "/image/side");
    std::filesystem::create_directories(base_folder + "/kinematic/PSM1");
    std::filesystem::create_directories(base_folder + "/kinematic/PSM2");
    std::filesystem::create_directories(base_folder + "/kinematic/ECM");
    std::filesystem::create_directories(base_folder + "/time_syn");

    std::vector<std::filesystem::path> folders;        

    for (const auto &entry : std::filesystem::directory_iterator("recorded_data"))
    {
        if (entry.is_directory())
            folders.emplace_back(entry.path());
    }

    auto timeLess = [](const std::filesystem::path &a,
                    const std::filesystem::path &b) -> bool
    {
        auto parse = [](const std::string &name) -> std::pair<long long,long long>
        {
            const size_t us = name.find('_');
            long long sec  = std::stoll(name.substr(0, us));
            long long nsec = (us == std::string::npos) ? 0
                                                    : std::stoll(name.substr(us + 1));
            return {sec, nsec};
        };

        const auto ta = parse(a.filename().string());
        const auto tb = parse(b.filename().string());

        return  (ta.first  < tb.first)  ||
            ((ta.first == tb.first) && ta.second < tb.second);
    };
    std::sort(folders.begin(), folders.end(), timeLess);


    int index = 0;
    for (const auto &dir : folders)  
         {
            std::string img_left_src   = dir.string() + "/image_left.png";
            std::string img_right_src  = dir.string() + "/image_right.png";
            std::string img_side_src   = dir.string() + "/image_side.png";
            std::string kin_src_psm1   = dir.string() + "/kinematics_PSM1.json";
            std::string kin_src_psm2   = dir.string() + "/kinematics_PSM2.json";
            std::string kin_src_ecm    = dir.string() + "/kinematics_ECM.json";
            std::string time_syn_src   = dir.string(); // the entire folder as context

            std::string img_left_dst   = base_folder + "/image/left/" + std::to_string(index) + ".png";
            std::string img_right_dst  = base_folder + "/image/right/" + std::to_string(index) + ".png";
            std::string img_side_dst   = base_folder + "/image/side/" + std::to_string(index) + ".png";

            // Now name the kinematics files "index_PSM1.json" and "index_PSM2.json" and "index_ECM.json"
            std::string kin_dst_psm1   = base_folder + "/kinematic/PSM1/" + std::to_string(index) + ".json";
            std::string kin_dst_psm2   = base_folder + "/kinematic/PSM2/" + std::to_string(index) + ".json";
            std::string kin_dst_ecm    = base_folder + "/kinematic/ECM/" + std::to_string(index) + ".json";

            std::string time_syn_dst   = base_folder + "/time_syn/" + std::to_string(index) + ".json";

            if (std::filesystem::exists(img_left_src)) {
                std::filesystem::copy(img_left_src, img_left_dst,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(img_right_src)) {
                std::filesystem::copy(img_right_src, img_right_dst,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(img_side_src)) {
                std::filesystem::copy(img_side_src, img_side_dst,
                                      std::filesystem::copy_options::overwrite_existing);
            }            
            if (std::filesystem::exists(kin_src_psm1)) {
                std::filesystem::copy(kin_src_psm1, kin_dst_psm1,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(kin_src_psm2)) {
                std::filesystem::copy(kin_src_psm2, kin_dst_psm2,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(kin_src_ecm)) {
                std::filesystem::copy(kin_src_ecm, kin_dst_ecm,
                                      std::filesystem::copy_options::overwrite_existing);
            }

            // Save time info
            Json::Value time_data;
            time_data["timestamp"] = dir.filename().string();
            std::ofstream file(time_syn_dst);
            if (file) {
                file << time_data.toStyledString();
                file.close();
            }

            index++;
        }
}

    // De-comment the line below to remove the recorded_data folder!
    // The line is commented below for testing purposes
    // std::filesystem::remove_all("recorded_data");





int main(int argc, char** argv) {
    std::cout << "------ program starting ------" << std::endl;

    // parse custom arguments BEFORE ros::init so ROS can consume its own
    parseArguments(argc, argv);

    ros::init(argc, argv, "synchronized_recorder_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber left_sub;
    image_transport::Subscriber right_sub;
    image_transport::Subscriber side_sub;

    if (g_use_left_image) {
        std::string left_topic = "/" + g_camera_topic_base + "/left/image_raw";
        left_sub  = it.subscribe(left_topic,  1, imageCallbackLeft);
    }
    if (g_use_right_image) {
        std::string right_topic = "/" + g_camera_topic_base + "/right/image_raw";
        right_sub = it.subscribe(right_topic, 1, imageCallbackRight);
    }
    if (g_use_side_image) {
    side_sub = it.subscribe("/camera/image_raw", 1, imageCallbackSide);
    }

    ros::Subscriber cp_sub_psm1;
    ros::Subscriber js_sub_psm1;
    ros::Subscriber cp_sub_psm2;
    ros::Subscriber js_sub_psm2;
    ros::Subscriber cp_sub_ecm;
    ros::Subscriber js_sub_ecm;

    ros::Subscriber cp_set_sub_psm1;
    ros::Subscriber cp_set_sub_psm2;
    ros::Subscriber cp_set_sub_ecm;
    ros::Subscriber js_set_sub_psm1;
    ros::Subscriber js_set_sub_psm2;
    ros::Subscriber js_set_sub_ecm;

    ros::Subscriber jaw_meas_sub_psm1;
    ros::Subscriber jaw_meas_sub_psm2;
    ros::Subscriber jaw_set_sub_psm1;
    ros::Subscriber jaw_set_sub_psm2;
    ros::Subscriber cv_sub_psm1, cv_sub_psm2;

    ros::Subscriber lcp_sub_psm1;
    ros::Subscriber lcp_sub_psm2;
    ros::Subscriber lcp_sub_ecm;

    // adding in a subscriber for PSM1
    if (g_record_psm1) {

            std::string js_topic_psm1 = "/PSM1/measured_js";
            js_sub_psm1 = nh.subscribe(js_topic_psm1, 1, jointStateCallback);

            std::string cp_topic_psm1 = "/PSM1/measured_cp";
            cp_sub_psm1 = nh.subscribe(cp_topic_psm1, 1, poseCallbackPSM1);

            std::string cp_set_topic_psm1 = "/PSM1/setpoint_cp";
            cp_set_sub_psm1 = nh.subscribe(cp_set_topic_psm1, 1, setpointCPCallbackPSM1);

            std::string js_set_topic_psm1 = "/PSM1/setpoint_js";
            js_set_sub_psm1 = nh.subscribe(js_set_topic_psm1, 1, setpointJSCallbackPSM1);

            std::string jaw_topic = "/PSM1/jaw/measured_js";
            jaw_meas_sub_psm1 = nh.subscribe(jaw_topic, 1, jawMeasuredJSCallbackPSM1);
            std::string jaw_set_topic = "/PSM1/jaw/setpoint_js";
            jaw_set_sub_psm1 = nh.subscribe(jaw_set_topic, 1, jawSetpointJSCallbackPSM1);

            lcp_sub_psm1 = nh.subscribe("/PSM1/local/measured_cp", 1, localCPCallbackPSM1);
    }

    // adding in a subscriber for PSM2
    if (g_record_psm2) {
 
            std::string js_topic_psm2 = "/PSM2/measured_js";
            js_sub_psm2 = nh.subscribe(js_topic_psm2, 1, jointStateCallbackPSM2);

            std::string cp_topic_psm2  = "/PSM2/measured_cp";
            cp_sub_psm2 = nh.subscribe(cp_topic_psm2, 1, poseCallbackPSM2);

            std::string js_set_topic_psm2 = "/PSM2/setpoint_js";
            js_set_sub_psm2 = nh.subscribe(js_set_topic_psm2, 1, setpointJSCallbackPSM2);

            std::string cp_set_topic_psm2 = "/PSM2/setpoint_cp";
            cp_set_sub_psm2 = nh.subscribe(cp_set_topic_psm2, 1, setpointCPCallbackPSM2);

            std::string jaw_topic = "/PSM2/jaw/measured_js";
            jaw_meas_sub_psm2 = nh.subscribe(jaw_topic, 1, jawMeasuredJSCallbackPSM2);

            std::string jaw_set_topic = "/PSM2/jaw/setpoint_js";
            jaw_set_sub_psm2 = nh.subscribe(jaw_set_topic, 1, jawSetpointJSCallbackPSM2);

            lcp_sub_psm2 = nh.subscribe("/PSM2/local/measured_cp", 1, localCPCallbackPSM2);

    }

    // adding in a subscriber for ECM
    if (g_record_ecm) {

            std::string js_topic_ecm = "/ECM/measured_js";
            js_sub_ecm = nh.subscribe(js_topic_ecm, 1, jointStateCallbackECM);

            std::string js_set_topic_ecm = "/ECM/setpoint_js";
            js_set_sub_ecm = nh.subscribe(js_set_topic_ecm, 1, setpointJSCallbackECM);

            std::string cp_topic_ecm = "/ECM/measured_cp";
            cp_sub_ecm = nh.subscribe(cp_topic_ecm, 1, poseCallbackECM);

            std::string cp_set_topic_ecm = "/ECM/setpoint_cp";
            cp_set_sub_ecm = nh.subscribe(cp_set_topic_ecm, 1, setpointCPCallbackECM);

            lcp_sub_ecm  = nh.subscribe("/ECM/local/measured_cp", 1,  localCPCallbackECM);
        
    }


    if (g_record_cv) {
            if (g_record_psm1)
                cv_sub_psm1 = nh.subscribe("/PSM1/measured_cv", 1, cvCallbackPSM1);
            if (g_record_psm2)
                cv_sub_psm2 = nh.subscribe("/PSM2/measured_cv", 1, cvCallbackPSM2);
    }

    std::cout << "------ start ROS spin in separate thread ------" << std::endl;
    std::thread ros_spin_thread([] {
        ros::spin();
        // Once spin() returns, shutdown
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    });


    std::thread sync_t(syncThread);

    // --- launch a pool of writer threads for higher throughput ---
    std::vector<std::thread> writer_threads;
    for (int i = 0; i < NUM_WRITER_THREADS; ++i) {
        writer_threads.emplace_back(writerThread);
    }

    ros_spin_thread.join();

    

    {
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    }

    // join threads
    sync_t.join();
    for (auto &t : writer_threads) {
        if (t.joinable()) t.join();
    }
 
    cleanupFolders();
    countFoldersPerSecond();

    
    reformatDataStorage();

    return 0;
}