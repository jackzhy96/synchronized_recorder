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

const size_t MAX_BUFFER_SIZE = 1000;
const double TIME_TOL = 0.005;   // 5ms tolerance

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ImageData img_data;
    img_data.stamp = msg->header.stamp;
    img_data.image = cv_ptr->image.clone();

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

    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_kinematic_buffer.size() > MAX_BUFFER_SIZE) {
            g_kinematic_buffer.pop();
        }
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
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

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

        // ------ below is an attempt to create separate threads for writing images -------
        // ------ It doesn't work! Do not enable. Will crash the dvrk console!! -------
        // std::thread([](std::string filename, cv::Mat img)
        // {
        //     cv::imwrite(filename,img);
        // },
        // img_filename, local_img.image).detach();


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


int main(int argc, char** argv) {
    std::cout << " ------ program starting ------" << std::endl;
    ros::init(argc, argv, "synchronized_recorder_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/test/right/image_raw", 1, imageCallback);

    ros::Subscriber joint_sub = nh.subscribe("/PSM1/measured_js", 1, jointStateCallback);

    std::cout << " ------ start ros spin ------" << std::endl;
    // Spin ROS callback in a separate thread
    std::thread ros_spin_thread([] {
        ros::spin();
        g_keep_running = false;
        g_cv.notify_all();
    });

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
    countFoldersPerSecond();


    return 0;
}
