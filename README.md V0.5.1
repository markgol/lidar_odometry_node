### Updated 2025-04-15 V0.5.0

lidar_odometry_node
===================

Overview
--------

`lidar_odometry_node` is a ROS2 node that performs **real-time LiDAR-based odometry** using an ICP (Iterative Closest Point) scan-to-submap alignment approach.

The node estimates the robot's motion incrementally by aligning incoming LiDAR scans to a **sliding local map**, producing a continuous odometry estimate in the `odom` frame.

This node is designed as the **front-end odometry component** of a larger SLAM system and publishes **keyframes** for downstream processing by a pose graph SLAM node.

* * *

Key Features
------------

* ICP-based scan-to-submap alignment

* Sliding local map for stable real-time performance

* Adaptive noise-aware motion estimation

* Optional distance-based cropping for ICP acceleration

* Keyframe extraction with adaptive thresholds

* ROS2-native publishers, subscribers, and TF integration

* * *

System Assumptions
------------------

This node assumes the following about the incoming data:

* LiDAR provides **full 360° field-of-view 3D point cloud**

* Point clouds are: **pose corrected using IMU quaternion**  (in the L2lidar_node)

* Input clouds are effectively aligned with the `base_link` frame

* * *

Node Responsibilities
---------------------

### Subscribes to

* `/points` (`sensor_msgs/msg/PointCloud2`)  
  LiDAR point cloud data

* `/imu` (`sensor_msgs/msg/Imu`)  
  IMU data (optional for future use)

* * *

### Publishes

Topic names are input from the config yaml file.  These are the default names.

* `/odom` (`nav_msgs/msg/Odometry`)  
  Incremental odometry estimate

* `/aligned_scan` (`sensor_msgs/msg/PointCloud2`)  
  ICP-aligned scan (diagnostic)

* `/path` (`nav_msgs/msg/Path`)  
  Odometry path (diagnostic)

* `/lidar/keyframes/pose` (`geometry_msgs/msg/PoseStamped`)  
  Extracted keyframe poses

* `/lidar/keyframes/cloud` (`sensor_msgs/msg/PointCloud2`)  
  Extracted keyframe point clouds (full scans)

* * *

### TF Frames

The node publishes:
    odom → base_link

Expected TF tree:
    map → odom → base_link → lidar

* * *

Processing Pipeline
-------------------

    Incoming Scan (/points)
            ↓
    Voxel Downsampling
            ↓
    Transform to base_link
            ↓
    (Optional) Crop for ICP
            ↓
    ICP Alignment (scan → local map)
            ↓
    Update Odometry (T_odom_base)
            ↓
    Update Local Map (sliding window)
            ↓
    Keyframe Extraction (full scan)
            ↓
    Publish outputs

* * *

Keyframe Extraction
-------------------

Keyframes are generated to support downstream SLAM (e.g., pose graph optimization).

### Key Properties

* Keyframes use the **full aligned scan**, not cropped data

* ICP results are applied to the full scan before storage

* Keyframes are published in the `odom` frame

* * *

### Keyframe Criteria

Keyframes are generated using:

* **Primary trigger:** translation threshold ( large distance move)

* **Secondary trigger:** rotation gated by minimum translation (small distance move)

* **Adaptive thresholding:** based on motion noise statistics

This avoids false positives caused by:

* IMU noise (IMU is currently not used here but noise can effect pose in the l2lidar_node)

* small ICP corrections

* stationary jitter

* * *

Parameters
----------

### ICP parameters

| Parameter         | Type   | Description                                         | default value |
| ----------------- | ------ | --------------------------------------------------- | ------------- |
| `voxel_leaf`      | double | Downsampling leaf size                              | 0.03          |
| `correspondence`  | double | ICP correspondence distance                         | 1.0           |
| `epsilon`         | double | ICP convergence threshold                           | 1.0e-6        |
| fitness_score     | double | ICP fitness threshold                               | 0.3           |
| icp_iterations    | int    | Max. number ICP iterations                          | 25            |
| max_noiseDistance | double | Max noise threshold for motion estimation in meters | 0.03          |

* * *

### Map parameters

| Parameter               | Type   | Description                                                                               | default value |
| ----------------------- | ------ | ----------------------------------------------------------------------------------------- | ------------- |
| init_scans              | int    | Number of scan in initial map                                                             | 20            |
| `local_map_max_size`    | int    | Maximum number of points in local map                                                     | 400000        |
| `scan_trim_size`        | int    | max number of last scans to use to reset the local map when it exceeds the local map size | 30            |
| `local_submap_distance` | double | Crop radius in meters  (if enabled)                                                       | 4.0           |
| EnableCropDistance      | bool   | Enable crop of local map                                                                  | true          |
| max_scan_queue          | int    | max scan queue size                                                                       | 45            |

* * *

### Keyframe parameters

| Parameter                               | Type   | Description                                       | default value |
| --------------------------------------- | ------ | ------------------------------------------------- | ------------- |
| `keyframe_translation_thresh`           | double | Translation threshold (meters)                    | 0.5           |
| `keyframe_rotation_thresh`              | double | Rotation threshold (radians)                      | 0.3           |
| `keyframe_min_translation_for_rotation` | double | Minimum translation required to consider rotation | 0.1           |
| keyframe_last_frame_time                | double | minimum keyframe publish rate in seconds          | 2.0           |

* * *

### Topic and frame parameters

| Parameter            | Type   | Description                               | default value          |
| -------------------- | ------ | ----------------------------------------- | ---------------------- |
| robot_frame_id       | string | robot frame                               | base_link              |
| odometry_frame_id    | string | odometry frame                            | odom                   |
| odom_topic           | string | odometry topic                            | /odom                  |
| keyframe_pose_topic  | string | keyframe robot pose topic                 | /lidar/keyframes/pose  |
| keyframe_point_topic | string | keyframe point cloud topic                | /lidar/keyframes/cloud |
| path_topic           | string | diagnostic local robot path topic         | /path                  |
| aligned_scan_topic   | string | diagnostic aligned scan point cloud topic | /aligned_scan          |

***

### Other parameters

| Parameter        | Type | Description                                                           | default value |
| ---------------- | ---- | --------------------------------------------------------------------- | ------------- |
| watchdog_timeout | int  | node stops if no subcription data received for this number of seconds | 60            |
|                  |      |                                                                       |               |
| max_imu_queue    | int  | FIFO queue of IMU packets (only if enabled)                           | 540           |

* * *

Local Map Behavior
------------------

* Maintains a **sliding window of recent scans**

* Limits computational cost of ICP

* Periodically trims and rebuilds based on scan queue

* Optionally cropped by distance for ICP matching to improve performance (only the local map is cropped, keyframes are without cropping)

* * *

Important Design Notes
----------------------

### 1. Cropped vs Full Scans

* **Cropped scans** are used ONLY for ICP input

* **Full scans** are used for:
  
  * keyframes
  
  * future SLAM processing

This separation ensures:

* fast alignment

* robust loop closure later

* * *

### 2. Odometry Frame

* All outputs are in the `odom` frame

* This frame is **locally consistent but drifts over time**

* Global correction will be handled by the SLAM node

* * *

### 3. Adaptive Motion Filtering

The node computes:

* mean translation (`avgDistance_`)

* variance (`sigmaDistance_`)

These are used to:

* suppress noise

* improve motion estimation

* support adaptive keyframe selection

* * *

Diagnostics
-----------

The node provides:

* `/aligned_scan` for ICP visualization

* `/path` for trajectory inspection

* console logs for:
  
  * ICP timing
  
  * fitness score
  
  * noise statistics

* * *

Limitations
-----------

* No loop closure

* No global map

* Drift accumulates over time

* * *

Integration
-----------

This node is intended to work with:

* `pose_graph_slam_node` (for global optimization)

* `global_map_builder_node` (for map construction)

* * *

Future Work
-----------

* Loop closure integration

* Pose graph optimization

* map → odom correction

* Multi-sensor fusion (e.g., wheel odometry, vision)

* * *

Build
-----

This package is intended to be built using **CMake (ament_cmake)**.
    colcon build --packages-select <your_package>

* * *

Run
---

    ros2 run <your_package> lidar_odometry_node

* * *

Author Notes
------------

This node is designed for **modular SLAM system integration**, emphasizing:

* real-time performance

* clean separation of processing concerns

* extensibility toward full SLAM

* This utilizes data pulished from the l2lidar_node V0.2.3

* * *

## Version

V0.5.0    2026-04-15    This is the initial public release.  It is operable but has not been fully tested.  It has further development that is being done.  This initial release is part of the projects larger over-all skeleton for SLAM operation.  It explcitily does not use IMU or other odometry sensors. It relies purely on ICP SE(3) matching to determine odometry.  It has additional noise filtering to detect when the platform is not moving.  This was done to reduce drift accumulation when the platform isn't moving.  It also has a radius crop of the local map in order to reduce processing requirements.  The node assumes that the plaform is stationary when it first starts.  This allows the node to create a local map of the platform's location.
