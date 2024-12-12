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
<<<<<<< HEAD
#include <map>
#include <filesystem> // should be C++17

=======
#include <filesystem> // C++17
>>>>>>> 4754361 (Upload of the script )

struct KinematicData {
    ros::Time stamp;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;
};

<<<<<<< HEAD


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


=======
struct ImageData {
    ros::Time stamp;
    cv::Mat image;
};

// Shared Data
std::mutex g_data_mutex;
std::condition_variable g_cv;
bool g_data_ready = false;
bool g_keep_running = true;

std::queue<ImageData> g_image_buffer;
std::queue<KinematicData> g_kinematic_buffer;

ImageData g_synced_image;
KinematicData g_synced_kinematic;

int g_writers_done = 0;  // How many writers have finished writing the current pair
std::string g_synced_folder;

const size_t MAX_BUFFER_SIZE = 100;
const double TIME_TOL = 0.0005;   // 1ms tolerance

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

>>>>>>> 4754361 (Upload of the script )
    ImageData img_data;
    img_data.stamp = msg->header.stamp;
    img_data.image = cv_ptr->image.clone();

<<<<<<< HEAD

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

    
=======
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_image_buffer.size() > MAX_BUFFER_SIZE) {
            g_image_buffer.pop();
        }
        g_image_buffer.push(img_data);
    }
}

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    KinematicData kin_data;
    kin_data.stamp = msg->header.stamp;
    kin_data.position = msg->position;
    kin_data.velocity = msg->velocity;
    kin_data.effort = msg->effort;

>>>>>>> 4754361 (Upload of the script )
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer.pop();
        }
<<<<<<< HEAD

        g_kinematic_buffer.push(kin_data);
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
        if (g_kinematic_buffer_psm2.size() > MAX_BUFFER_SIZE) {
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

            if (!g_left_image_buffer.empty())    g_left_image_buffer.pop();
            if (!g_right_image_buffer.empty())   g_right_image_buffer.pop();
            if (!g_kinematic_buffer.empty())     g_kinematic_buffer.pop();
            if (!g_kinematic_buffer_psm2.empty()) g_kinematic_buffer_psm2.pop();
            continue;


        }

        // if any buffer is empty, can't sync
        if (g_left_image_buffer.empty() ||
            g_right_image_buffer.empty() ||
            g_kinematic_buffer.empty() ||            // PSM1
            g_kinematic_buffer_psm2.empty()) {       // PSM2
            continue;
        }

        
        ImageData     left_img  = g_left_image_buffer.front();
        ImageData     right_img = g_right_image_buffer.front();
        KinematicData kin1      = g_kinematic_buffer.front();       // PSM1
        KinematicData kin2      = g_kinematic_buffer_psm2.front();  // PSM2


        
        double time_diff_left  = fabs((left_img.stamp   - kin1.stamp).toSec());
        double time_diff_right = fabs((right_img.stamp  - kin1.stamp).toSec());
        double time_diff_kin2  = fabs((kin2.stamp       - kin1.stamp).toSec());



        if (time_diff_left < TIME_TOL &&
            time_diff_right < TIME_TOL &&
            time_diff_kin2 < TIME_TOL) 
        {


            SyncedPacket packet;

            packet.stamp     = kin1.stamp; 
            packet.left_img  = left_img;
            packet.right_img = right_img;
            packet.kin_psm1  = kin1;
            packet.kin_psm2  = kin2;


            g_synced_queue.push(packet);


            // pop them all
            g_left_image_buffer.pop();
            g_right_image_buffer.pop();
            g_kinematic_buffer.pop();       // PSM1
            g_kinematic_buffer_psm2.pop();  // PSM2

            g_cv.notify_one();
        } 
        
        
        else {
            
            ros::Time oldest_stamp = left_img.stamp;
            if (right_img.stamp < oldest_stamp) oldest_stamp = right_img.stamp;
            if (kin1.stamp < oldest_stamp)      oldest_stamp = kin1.stamp;
            if (kin2.stamp < oldest_stamp)      oldest_stamp = kin2.stamp;

            if (left_img.stamp == oldest_stamp) {
                g_left_image_buffer.pop();
            } 

            else if (right_img.stamp == oldest_stamp) {
                g_right_image_buffer.pop();
            } 
            else if (kin1.stamp == oldest_stamp) {
                g_kinematic_buffer.pop();        // PSM1
            } 
            else {
                g_kinematic_buffer_psm2.pop();   // PSM2
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
        g_cv.wait(lock, [] {
            return !g_synced_queue.empty() || !g_keep_running;
        });


        if (!g_keep_running && g_synced_queue.empty()) {
            break;
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




        // --- Write PSM1 kinematics ---
        {

            std::string kin_path_psm1 = final_folder + "/kinematics_PSM1.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.kin_psm1.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.kin_psm1.stamp.nsec;


            Json::Value pos(Json::arrayValue);
            for (auto &p : packet.kin_psm1.position) { pos.append(p); }
            root["position"] = pos;


            Json::Value vel(Json::arrayValue);
            for (auto &v : packet.kin_psm1.velocity) { vel.append(v); }
            root["velocity"] = vel;


            Json::Value eff(Json::arrayValue);
            for (auto &e : packet.kin_psm1.effort) { eff.append(e); }
            root["effort"] = eff;


            std::ofstream file(kin_path_psm1);
            if (!file) {
                std::cerr << "Failed to open kinematics_PSM1.json for writing.\n";
                continue;
            }


            file << root.toStyledString();
            file.close();
        }

        // --- Write PSM2 kinematics ---
        {

            std::string kin_path_psm2 = final_folder + "/kinematics_PSM2.json";
            Json::Value root;
            root["header"]["sec"]  = (Json::Value::Int64)packet.kin_psm2.stamp.sec;
            root["header"]["nsec"] = (Json::Value::Int64)packet.kin_psm2.stamp.nsec;


            Json::Value pos(Json::arrayValue);
            for (auto &p : packet.kin_psm2.position) { pos.append(p); }
            root["position"] = pos;


            Json::Value vel(Json::arrayValue);
            for (auto &v : packet.kin_psm2.velocity) { vel.append(v); }
            root["velocity"] = vel;


            Json::Value eff(Json::arrayValue);
            for (auto &e : packet.kin_psm2.effort) { eff.append(e); }
            root["effort"] = eff;


            std::ofstream file(kin_path_psm2);
            if (!file) {
                std::cerr << "Failed to open kinematics_PSM2.json for writing.\n";
                continue;
            }

            file << root.toStyledString();
            file.close();
        }


        std::cout << "[writerThread] Wrote images + PSM1 & PSM2 JSON to " 
                  << final_folder << std::endl;
    }


    std::cout << "------ writerThread exiting ------" << std::endl;
}





// ---------------------------- Post-processing functions below ----------------------------


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
            auto json_psm1_path = entry.path() / "kinematics_PSM1.json";
            auto json_psm2_path = entry.path() / "kinematics_PSM2.json";


            bool left_exists   = std::filesystem::exists(img_left_path);
            bool right_exists  = std::filesystem::exists(img_right_path);
            bool psm1_exists   = std::filesystem::exists(json_psm1_path);
            bool psm2_exists   = std::filesystem::exists(json_psm2_path);




            // If missing any of the 4, remove the folder
            if (!left_exists || !right_exists || !psm1_exists || !psm2_exists) {
                std::cout << "Removing incomplete folder: " 
                          << entry.path().string() << std::endl;
=======
        g_kinematic_buffer.push(kin_data);
    }
}

// A helper function to be called by writers after finishing their job
void writerDone() {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_writers_done++;
    if (g_writers_done == 2) {
        // Both writers have finished successfully
        g_data_ready = false;
        g_writers_done = 0;
    }
}

// Thread #2: Synchronization
void syncThread() {
    std::cout << " ------ entered sync thread ------" << std::endl;
    while (ros::ok() && g_keep_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_data_ready) {
            // Already have data ready, skip
            continue;
        }

        if (g_image_buffer.empty() || g_kinematic_buffer.empty()) {
            continue;
        }

        ImageData img_data = g_image_buffer.front();
        KinematicData kin_data = g_kinematic_buffer.front();
        double time_diff = fabs((img_data.stamp - kin_data.stamp).toSec());
        if (time_diff < TIME_TOL) {
            g_synced_image = img_data;
            g_synced_kinematic = kin_data;

            g_image_buffer.pop();
            g_kinematic_buffer.pop();

            // Create a folder named after the timestamp of this pair
            std::ostringstream folder_name;
            folder_name << "recorded_data/" << g_synced_image.stamp.sec << "_" << g_synced_image.stamp.nsec;
            g_synced_folder = folder_name.str();

            std::error_code ec;
            std::filesystem::create_directories(g_synced_folder, ec);
            if (ec) {
                std::cerr << "Failed to create directory " << g_synced_folder << ": " << ec.message() << std::endl;
                g_data_ready = false;
                continue;
            }

            g_data_ready = true;
            g_cv.notify_all();
        } else {
            // Discard the older one
            if (img_data.stamp < kin_data.stamp) {
                g_image_buffer.pop();
            } else {
                g_kinematic_buffer.pop();
            }
        }
    }
}

// Thread #3: Image Writer
void imageWriterThread() {
    std::cout << " ------ entered imageWriter thread ------" << std::endl;
    while (ros::ok() && g_keep_running) {
        std::unique_lock<std::mutex> lock(g_data_mutex);
        g_cv.wait(lock, [] { return g_data_ready || !g_keep_running; });
        if (!g_keep_running) break;

        ImageData local_img = g_synced_image; // Make a copy
        std::string folder = g_synced_folder;
        lock.unlock();

        std::string img_filename = folder + "/image.png";
        bool success = cv::imwrite(img_filename, local_img.image);
        if (success) {
            std::cout << " ****** wrote a sync image to " << img_filename << " ****** " << std::endl;
            writerDone(); 
        } else {
            std::cerr << "Failed to write image: " << img_filename << std::endl;
            {
                std::lock_guard<std::mutex> fail_lock(g_data_mutex);
                g_data_ready = false;
                g_writers_done = 0;
            }
        }
    }
}

// Thread #4: Kinematic Writer
void kinematicWriterThread() {
    std::cout << " ------ entered kinematicWriter thread ------" << std::endl;
    while (ros::ok() && g_keep_running) {
        std::unique_lock<std::mutex> lock(g_data_mutex);
        g_cv.wait(lock, [] { return g_data_ready || !g_keep_running; });
        if (!g_keep_running) break;

        KinematicData local_kin = g_synced_kinematic; // Make a copy
        std::string folder = g_synced_folder;
        lock.unlock();

        std::string kin_filename = folder + "/kinematics.json";

        Json::Value root;
        root["header"]["sec"] = (Json::Value::Int64)local_kin.stamp.sec;
        root["header"]["nsec"] = (Json::Value::Int64)local_kin.stamp.nsec;

        Json::Value pos(Json::arrayValue);
        for (auto &p : local_kin.position) {
            pos.append(p);
        }
        root["position"] = pos;

        Json::Value vel(Json::arrayValue);
        for (auto &v : local_kin.velocity) {
            vel.append(v);
        }
        root["velocity"] = vel;

        Json::Value eff(Json::arrayValue);
        for (auto &e : local_kin.effort) {
            eff.append(e);
        }
        root["effort"] = eff;

        std::ofstream file(kin_filename);
        if (file) {
            file << root.toStyledString();
            file.close();
            std::cout << " ****** wrote a sync json to " << kin_filename << " ****** " << std::endl;
            writerDone();
        } else {
            std::cerr << "Failed to write json: " << kin_filename << std::endl;
            {
                std::lock_guard<std::mutex> fail_lock(g_data_mutex);
                g_data_ready = false;
                g_writers_done = 0;
            }
        }
    }
}

void cleanupFolders() {
    std::cout << " ------ cleanup step: checking for incomplete folders ------" << std::endl;
    std::filesystem::path base_dir("recorded_data");
    if (!std::filesystem::exists(base_dir)) {
        return; // No recorded_data directory
    }

    for (auto &entry : std::filesystem::directory_iterator(base_dir)) {
        if (entry.is_directory()) {
            auto img_path = entry.path() / "image.png";
            auto json_path = entry.path() / "kinematics.json";
            bool img_exists = std::filesystem::exists(img_path);
            bool json_exists = std::filesystem::exists(json_path);

            // If folder has only JSON and no image, delete the folder
            if (!img_exists && json_exists) {
                std::cout << "Removing incomplete folder: " << entry.path().string() << std::endl;
>>>>>>> 4754361 (Upload of the script )
                std::filesystem::remove_all(entry.path());
            }
        }
    }
}

<<<<<<< HEAD


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
            std::string img_left_src   = entry.path().string() + "/image_left.png";
            std::string img_right_src  = entry.path().string() + "/image_right.png";
            std::string kin_src_psm1   = entry.path().string() + "/kinematics_PSM1.json";
            std::string kin_src_psm2   = entry.path().string() + "/kinematics_PSM2.json";
            std::string time_syn_src   = entry.path().string(); // the entire folder as context

            std::string img_left_dst   = base_folder + "/image/" + std::to_string(index) + "_left.png";
            std::string img_right_dst  = base_folder + "/image/" + std::to_string(index) + "_right.png";
            // Now name the kinematics files "index_PSM1.json" and "index_PSM2.json"
            std::string kin_dst_psm1   = base_folder + "/kinematic/" + std::to_string(index) + "_PSM1.json";
            std::string kin_dst_psm2   = base_folder + "/kinematic/" + std::to_string(index) + "_PSM2.json";

            std::string time_syn_dst   = base_folder + "/time_syn/" + std::to_string(index) + ".json";

            if (std::filesystem::exists(img_left_src)) {
                std::filesystem::copy(img_left_src, img_left_dst,
                                      std::filesystem::copy_options::overwrite_existing);
            }
            if (std::filesystem::exists(img_right_src)) {
                std::filesystem::copy(img_right_src, img_right_dst,
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

            // Save time info
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
=======
int main(int argc, char** argv) {
    std::cout << " ------ program starting ------" << std::endl;
>>>>>>> 4754361 (Upload of the script )
    ros::init(argc, argv, "synchronized_recorder_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
<<<<<<< HEAD

    image_transport::Subscriber left_sub  = it.subscribe("/test/left/image_raw",  1, imageCallbackLeft);
    image_transport::Subscriber right_sub = it.subscribe("/test/right/image_raw", 1, imageCallbackRight);


    ros::Subscriber joint_sub_psm1 = nh.subscribe("/PSM1/measured_js", 1, jointStateCallback);

    // adding in a subscriber for PSM2
    ros::Subscriber joint_sub_psm2 = nh.subscribe("/PSM2/measured_js", 1, jointStateCallbackPSM2);



    std::cout << "------ start ROS spin in separate thread ------" << std::endl;
    std::thread ros_spin_thread([] {
        ros::spin();
        // Once spin() returns, shutdown
        std::lock_guard<std::mutex> lk(g_data_mutex);
=======
    image_transport::Subscriber image_sub = it.subscribe("/test/right/image_raw", 1, imageCallback);

    ros::Subscriber joint_sub = nh.subscribe("/PSM1/measured_js", 1, jointStateCallback);

    std::cout << " ------ start ros spin ------" << std::endl;
    // Spin ROS callback in a separate thread
    std::thread ros_spin_thread([] {
        ros::spin();
>>>>>>> 4754361 (Upload of the script )
        g_keep_running = false;
        g_cv.notify_all();
    });

<<<<<<< HEAD

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
=======
    std::cout << " ------ start threads ------" << std::endl;
    std::thread sync_t(syncThread);
    std::thread img_writer_t(imageWriterThread);
    std::thread kin_writer_t(kinematicWriterThread);

    ros_spin_thread.join();
    g_keep_running = false;
    g_cv.notify_all();

    sync_t.join();
    img_writer_t.join();
    kin_writer_t.join();

    // Perform cleanup after everything is done
    cleanupFolders();
>>>>>>> 4754361 (Upload of the script )

    return 0;
}
