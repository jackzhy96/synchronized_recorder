<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

# synchronized_recorder
synchronized recorder for image and kinematic data through ROS
A ROS node that synchronizes and records images and kinematic data. The synchronized data pairs are saved in organized folders with timestamps as folder names. Each folder contains:
1. A synchronized image (image.png)
2. Corresponding kinematic data in JSON format (kinematics.json)

## Main Contributor

Kevin Wu, Haoying(Jack) Zhou

## Note!!! With this current script, both PSM1 and PSM2 must be up and running!
=======
## IMPORTANT
With the current script **both PSM1 and PSM2 must be powered on and publishing** (even if you do not intend to save their data).  
However, the ECM arm is optional and can be toggled with `-a ECM`.
>>>>>>> d38f6e4 (Update README.md)

## File Structure
- Source Code: synchronized_recorder_node.cpp
- Output Directory: recorded_data/ (generated automatically in the working directory).

## Program Parameters
Topics: Update the topic names in the main() function to match the current daVinci setup.
Buffer Size: Adjust MAX_BUFFER_SIZE to control the size of data buffers.
Time Tolerance: Adjust TIME_TOL to specify synchronization accuracy.

## Build & Run Package
1. Clone repository
```
git clone <repository_url>
cd <repository_directory>
```
2. Build package
Ensure your CMakeLists.txt includes the necessary dependencies (ROS, OpenCV, JsonCpp). If something's missing, you'll know from error msgs during build.
```
catkin build --summary
source devel/setup.bash
```

3. Run node
Ensure that you've started all necessary processes (roscore, stereo camera, lighting if necessary).
Here'a full list of all dependencies necessary for this node:
  - Stereo camera. Start this using the following command:
    ```
    roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=test
    ```
    Important: stereo_rig_name must be set to "test"!!!
    
  - roscore

  - dvrk control console turned on.
    First, navigate to this folder: username@lcsr-dvrk-xx: ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci
    Start this using the following command:
    ```
    rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM-PSM1-PSM2.json -p 0.001
    ```
    Important: verify that the dvrk is in the powered on status in the console!
    
  - Verify there's output kinetic data, using these commands:
    ```
    rostopic echo /PSM1/measured_js

    rostopic echo /PSM2/measured_js
    ```
    If running measured_js doesn't produce the desired kinematic data output, try fixing this issue by clicking on "Home" button in the control panel.


## How this works
### Threads
- ROS Spin Thread: Handles ROS callbacks for receiving data.
- Synchronization Thread: Matches image and kinematic data based on timestamps.
- Image Writer Thread: Writes synchronized images to disk.
- Kinematic Writer Thread: Serializes and writes kinematic data in JSON format.
### Workflow
- Callbacks: Subscribe to ROS topics for image and kinematic data.
- Synchronization: Match data with a timestamp tolerance of 1ms.
- Output: Save synchronized data to local storage in timestamped folders.
- Cleanup: Remove incomplete folders to ensure data consistency.

## (Automatic) Double Checking Completeness of Data Pairs
If a folder is incomplete (e.g., missing image.png or kinematics.json), the folder would be removed during cleanup.
=======
=======
## Note!!! With this current script, both PSM1 and PSM2 must be up and running!

>>>>>>> 28deae8 (Update README.md)
# SynchronizedRecorder
A multi‑threaded ROS node that records, synchronizes, and stores:

* stereo or mono images from the da Vinci stereo camera, and
* da Vinci arm kinematics (measured + set‑point + jaw channels).

Each synchronized bundle is written to a timestamp‑named folder in the output directory (see below).

## File Structure
- Source Code: synchronized_recorder_node.cpp
- Output Directory: recorded_data/ (generated automatically in the working directory).


## Build & Run Package
1. **Clone repository**
```
git clone <repository_url>
cd <repository_directory>
```
2. **Build package**<br>
Ensure your CMakeLists.txt includes the necessary dependencies (ROS, OpenCV, JsonCpp). If something's missing, you'll know from error msgs during build.
```
catkin build --summary
source devel/setup.bash
```

3. **Run node**<br>
Ensure that you've started all necessary processes (roscore, stereo camera, lighting if necessary).<br>
Here's a full list of all dependencies necessary for this node:
  - Stereo camera. Start this using the following command:
    ```
    roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=test
    ```
    Important: stereo_rig_name can be set to whatever you'd like (doesn't necessarily need to be "test"), but you'll need to indicate the stereo_rig_name when passing in the <camera_topic> argument.
    
  - Make sure ```roscore``` is running.

  - Make sure dvrk control console is turned on.
    First, navigate to this folder:<br>
    `username@lcsr-dvrk-xx: ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci`<br>
    Start the console using the following command:
    ```
    rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM-PSM1-PSM2.json -p 0.001
    ```
    Important: verify that the dvrk is in the powered on status in the console!
    
  - Verify there's output kinematic data, using these commands:
    ```
    rostopic echo /PSM1/measured_js

    rostopic echo /PSM2/measured_js
    ```
    If running measured_js doesn't produce the desired kinematic data output, try fixing this issue by clicking on "Home" button in the control panel.
  - Run the script following the command below:
    ```
    -c <camera topic> -m <stereo|mono> [-d <left|right>] -a PSM1 [-a PSM2] [-a ECM] -x <js|cp> -t <time_tolerance_seconds>
    ```
    Explanation of arguments listed above:
    | Flag | Values | Required | Meaning |
    |------|--------|----------|---------|
    | `-c` | `<base_topic>` | yes | Base name of the camera rig |
    | `-m` | `stereo` \| `mono` | yes | Whether to record stereo (both cameras) or just one cam |
    | `-d` | `left` \| `right` | depends | Only when `-m mono`; picks which side image you want |
    | `-a` | `PSM1`, `PSM2`, `ECM` | yes (≥1) | Pick any subset of arms to record (repeat flag) |
    | `-x` | `js` \| `cp` | yes | Record joint‑space (`measured_js`) or Cartesian (`measured_cp`) streams |
    | `-t` | `<seconds>` | yes | Max time skew allowed when matching topics |
    
    Example (stereo, PSM1+PSM2, joint‑space, 2 ms tolerance):
    
    ```bash
    rosrun synchronized_recorder synchronized_recorder_node -c test -m stereo -a PSM1 -a PSM2 -x js -t 0.002
    

<br>

<br>


# How This Works
## Threads

| Thread               | What This Does                                                                                                                                                                   | Notes                                                                                                                   |
|----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|
| **ROS Spin**         | • Dispatches every subscriber callback.<br>• Feeds incoming messages into the per-topic buffers.                                                                                   | Runs in its own `std::thread` so the application can process data while ROS continues to spin.                          |
| **Sync Thread**      | • Dequeues the front-most element from each active buffer.<br>• Checks whether each timestamp lies within `±g_time_tol` of the reference.<br>• If **in tolerance**, packages all data into a `SyncedPacket` and pushes to `g_synced_queue`.<br>• If **not in tolerance**, drops the oldest message to maintain forward progress. | Runs in a tight loop protected by a single mutex.       |
| **Writer Thread** <br>(varying numbers, depending on user needs)| • Blocks on `g_synced_queue` until a packet is available.<br>• Pops one `SyncedPacket` and: <br>  1. Converts & writes image(s) as minimally compressed PNG .<br>  2. Serializes & writes kinematics to JSON.<br>  3. Emits any additional metadata we don't need. | Each thread writes independently. |


## Post-Proceeing
After ROS spin shuts down, execute the following post-processing steps:
* `cleanupFolders()` - removes any folder missing any required file.
* `countFoldersPerSecond()` - outputs a “effective Hz” statistic for frequency testing/optimization purposes
* `reformatDataStorage()` - puts data into the desired file format.



# 4  Extending the Recorder - Recording Additional ROS Topics

*Suppose you want to capture `/sample_topic` alongside everything else.*

1. **Add a buffer and callback** <br>
   Below, we're showing a template callback function for recording additional images.<br>
   If you want to record some type of kinematic instead, then you'll need to
   define a struct that carries the desired data. `<ImageData>` in the example below should be replaced with whatever struct you newly define.<br>
   You'll also need to define your own callback function (see step 4 below). 
   ```cpp
   std::queue<ImageData> g_sample_buffer;

   void sampleCallback(const sensor_msgs::ImageConstPtr& msg) {
       ImageData d;
       d.stamp = msg->header.stamp;
       d.image = cv_bridge::toCvShare(msg)->image.clone();
       std::lock_guard<std::mutex> lk(g_data_mutex);
       if (g_sample_buffer.size() > MAX_BUFFER_SIZE) g_sample_buffer.pop();
       g_sample_buffer.push(d);
   }

  3. **Subscribe in main()**
  ```
  image_transport::Subscriber sample_sub =
  it.subscribe("/sample_topic", 1, sampleCallback);
  ```

  3. **Extend argument list**<br>
  Add a CLI flag (e.g. -s sample) to toggle the recording of /sample_topic.

  4. **Update the synchronizer**
  * Add an `ImageData sample_img;` field to `SyncedPacket`.
  * Adjust the “any required buffer empty” check:
  ```if (g_record_sample && g_sample_buffer.empty()) continue;```
  * Include the newly added topic in the time tolerance test:
  ```in_tol &= fabs((sample_img.stamp - ref_stamp).toSec()) < g_time_tol;```
  * Pop the buffer when a packet is received:
  ```if (g_record_sample) g_sample_buffer.pop();```

  4. **Callback Function / Writer thread**<br>
  How you'll define the callback function and the writer thread for `/sample_topic` is dependent on how the `/sample_topic` data is structured!
  Each topic could have a different output structure, thus we cannot provide you with a universal template here.<br>
  However, you can refer to the existing callback functions and writer threads when defining your own. The existing callback functions for kinematic data is a great starting point - you may not need to chance that many things.

  5. **Clean-up & reformatter**<br>
  Add your new output file generated from `/sample_topic` to both `cleanupFolders()` and `reformatDataStorage()` so incomplete folders are still removed and data storage is formatted nicely.




>>>>>>> 7250110 (Update README.md)
