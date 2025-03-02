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
    KinematicData    kin;
};


std::mutex              g_data_mutex;
std::condition_variable g_cv;
bool                    g_keep_running = true;


// buffers for left image, right image, and kinematics
std::queue<ImageData>    g_left_image_buffer;
std::queue<ImageData>    g_right_image_buffer;
std::queue<KinematicData> g_kinematic_buffer;



// a queue for matched/synced data that needs to be written
std::queue<SyncedPacket> g_synced_queue;

const size_t MAX_BUFFER_SIZE         = 1000;     //
const size_t MAX_SYNCED_QUEUE_SIZE   = 100;      // queue for writing synced data
const double TIME_TOL                = 0.005;    // 5ms tolerance


// created separate call back functions for left and right stereo camera
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

// created separate call back functions for left and right stereo camera
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




void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp   = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort   = msg->effort;

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer.pop();
        }
        g_kinematic_buffer.push(kin_data);
    }
}

// Thread #2: Synchronization
void syncThread() {

    std::cout << "------ syncThread started ------" << std::endl;
    while (ros::ok() && g_keep_running) {
        std::lock_guard<std::mutex> lock(g_data_mutex);

        // If queue size exceeds threshold, skip data (doesn't happen usually)
        if (g_synced_queue.size() >= MAX_SYNCED_QUEUE_SIZE) {



            // drop the oldest data from input buffers
            
            if (!g_left_image_buffer.empty()) {
                g_left_image_buffer.pop();
            }
            if (!g_right_image_buffer.empty()) {
                g_right_image_buffer.pop();
            }
            if (!g_kinematic_buffer.empty()) {
                g_kinematic_buffer.pop();
            }
            continue;

        }


        if (g_left_image_buffer.empty() || g_right_image_buffer.empty() || g_kinematic_buffer.empty()) {
            continue;
        }

        

        ImageData left_img  = g_left_image_buffer.front();
        ImageData right_img = g_right_image_buffer.front();
        KinematicData kin_data = g_kinematic_buffer.front();

        
        double time_diff_left  = fabs((left_img.stamp  - kin_data.stamp).toSec());
        double time_diff_right = fabs((right_img.stamp - kin_data.stamp).toSec());

        
        // check if both left & right images are close in time with the kin data
        if (time_diff_left < TIME_TOL && time_diff_right < TIME_TOL) {

            // if they match, queue this packet up for writin

            SyncedPacket packet;

            packet.stamp     = kin_data.stamp; 
            packet.left_img  = left_img;
            packet.right_img = right_img;
            packet.kin       = kin_data;

            g_synced_queue.push(packet);



            g_left_image_buffer.pop();
            g_right_image_buffer.pop();
            g_kinematic_buffer.pop();

            // notify the writer thread
            g_cv.notify_one();


        } else {        // when the time doesn't match...

            
            ros::Time oldest_stamp = left_img.stamp;
            if (right_img.stamp < oldest_stamp) {
                oldest_stamp = right_img.stamp;
            }
            if (kin_data.stamp < oldest_stamp) {
                oldest_stamp = kin_data.stamp;
            }

            if (left_img.stamp == oldest_stamp) {
                g_left_image_buffer.pop();
            } else if (right_img.stamp == oldest_stamp) {
                g_right_image_buffer.pop();
            } else {
                g_kinematic_buffer.pop();
            }
        }
    }


    std::cout << "------ syncThread exiting ------" << std::endl;

}






// Thread #3: Image Writer: combined writing -see packet set-up above
void writerThread() {


    std::cout << "------ writerThread started ------" << std::endl;

    while (ros::ok() && g_keep_running) {
        std::unique_lock<std::mutex> lock(g_data_mutex);
        // Wait until something is in the queue or we are shutting down
        g_cv.wait(lock, [] { 
            return !g_synced_queue.empty() || !g_keep_running; 
        });

        if (!g_keep_running && g_synced_queue.empty()) {
            break; // done
        }



        SyncedPacket packet = g_synced_queue.front();
        g_synced_queue.pop();

        lock.unlock();  

        
        
        std::ostringstream folder_name;
        folder_name << "recorded_data/"
                    << packet.stamp.sec << "_" 
                    << packet.stamp.nsec;
        std::string final_folder = folder_name.str();

        
        std::error_code ec;
        std::filesystem::create_directories(final_folder, ec);
        if (ec) {
            std::cerr << "Failed to create directory " << final_folder 
                      << ": " << ec.message() << std::endl;
            continue;
        }



        // Convert from BGR -> RGB as requested

        cv::Mat left_rgb, right_rgb;
        cv::cvtColor(packet.left_img.image,  left_rgb,  cv::COLOR_BGR2RGB);
        cv::cvtColor(packet.right_img.image, right_rgb, cv::COLOR_BGR2RGB);




        std::string left_img_path  = final_folder + "/image_left.png";
        std::string right_img_path = final_folder + "/image_right.png";


        bool ok_left  = cv::imwrite(left_img_path,  left_rgb);
        bool ok_right = cv::imwrite(right_img_path, right_rgb);


        if (!ok_left || !ok_right) {
            std::cerr << "Failed to write images in " << final_folder << std::endl;
            continue;
        }

        // below writes kin data

        std::string kin_path = final_folder + "/kinematics.json";
        Json::Value root;
        root["header"]["sec"]  = (Json::Value::Int64)packet.kin.stamp.sec;
        root["header"]["nsec"] = (Json::Value::Int64)packet.kin.stamp.nsec;

        Json::Value pos(Json::arrayValue);
        for (auto &p : packet.kin.position) { pos.append(p); }
        root["position"] = pos;

        Json::Value vel(Json::arrayValue);
        for (auto &v : packet.kin.velocity) { vel.append(v); }
        root["velocity"] = vel;

        Json::Value eff(Json::arrayValue);
        for (auto &e : packet.kin.effort) { eff.append(e); }
        root["effort"] = eff;

        std::ofstream file(kin_path);
        if (!file) {
            std::cerr << "Failed to open kinematics.json for writing: " << kin_path << std::endl;
            continue;
        }
        file << root.toStyledString();
        file.close();

        std::cout << "[writerThread] Wrote images + JSON to " << final_folder << std::endl;

    }

    std::cout << "------ writerThread exiting ------" << std::endl;

}


// ---------------------------- Post-processing functions below ----------------------------
// 
 
void cleanupFolders() {
    std::cout << "------ cleanup step: checking for incomplete folders ------" << std::endl;
    std::filesystem::path base_dir("recorded_data");
    if (!std::filesystem::exists(base_dir)) {
        return; // No recorded_data directory
    }

    for (auto &entry : std::filesystem::directory_iterator(base_dir)) {
        if (entry.is_directory()) {
            auto img_left_path  = entry.path() / "image_left.png";
            auto img_right_path = entry.path() / "image_right.png";
            auto json_path      = entry.path() / "kinematics.json";

            bool left_exists  = std::filesystem::exists(img_left_path);
            bool right_exists = std::filesystem::exists(img_right_path);
            bool json_exists  = std::filesystem::exists(json_path);

            // If the folder does not have all three, remove it
            if (!left_exists || !right_exists || !json_exists) {
                std::cout << "Removing incomplete folder: " << entry.path().string() << std::endl;
                std::filesystem::remove_all(entry.path());
            }
        }
    }
}

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
    std::filesystem::create_directories(base_folder + "/image");
    std::filesystem::create_directories(base_folder + "/kinematic");
    std::filesystem::create_directories(base_folder + "/time_syn");

    int index = 0;
    for (const auto &entry : std::filesystem::directory_iterator("recorded_data")) {
        if (entry.is_directory()) {
            std::string img_src = entry.path().string() + "/image.png";
            std::string kin_src = entry.path().string() + "/kinematics.json";
            std::string time_syn_src = entry.path().string();

            std::string img_dst = base_folder + "/image/" + std::to_string(index) + ".jpg";
            std::string kin_dst = base_folder + "/kinematic/" + std::to_string(index) + ".json";
            std::string time_syn_dst = base_folder + "/time_syn/" + std::to_string(index) + ".json";

            if (std::filesystem::exists(img_src)) {
                std::filesystem::copy(img_src, img_dst, std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(kin_src)) {
                std::filesystem::copy(kin_src, kin_dst, std::filesystem::copy_options::overwrite_existing);
            }
            
            Json::Value time_data;
            time_data["timestamp"] = entry.path().filename().string();
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
}






int main(int argc, char** argv) {
    std::cout << "------ program starting ------" << std::endl;
    ros::init(argc, argv, "synchronized_recorder_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber left_sub  = it.subscribe("/test/left/image_raw",  1, imageCallbackLeft);
    image_transport::Subscriber right_sub = it.subscribe("/test/right/image_raw", 1, imageCallbackRight);

    
    ros::Subscriber joint_sub = nh.subscribe("/PSM1/measured_js", 1, jointStateCallback);


    std::cout << "------ start ROS spin in separate thread ------" << std::endl;
    std::thread ros_spin_thread([] {
        ros::spin();
        // Once spin() returns, shutdown
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    });


    std::thread sync_t(syncThread);

    std::thread writer_t(writerThread);

    ros_spin_thread.join();

    

    {
        std::lock_guard<std::mutex> lk(g_data_mutex);
        g_keep_running = false;
        g_cv.notify_all();
    }

    // join threads
    sync_t.join();
    writer_t.join();
 
    cleanupFolders();
    countFoldersPerSecond();

    
    reformatDataStorage();

    return 0;
}
