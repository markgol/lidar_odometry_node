### Updated 2025-04-05 V0.5.0

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

* IMU-assisted rotation correction (external to this node)

* ROS2-native publishers, subscribers, and TF integration

* * *

System Assumptions
------------------

This node assumes the following about the incoming data:

* LiDAR provides **full 360° field-of-view**

* Point clouds are:
  
  * **gravity-aligned**
  
  * **rotation-corrected** using IMU (upstream)

* Input clouds are effectively aligned with the `base_link` frame (rotation-wise)

* ICP primarily estimates **translational motion**, with only minor rotational refinement

* * *

Node Responsibilities
---------------------

### Subscribes to

* `/points` (`sensor_msgs/msg/PointCloud2`)  
  LiDAR point cloud data

* `/imu` (`sensor_msgs/msg/Imu`)  
  IMU data (used upstream for rotation correction)

* * *

### Publishes

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

* **Primary trigger:** translation threshold

* **Secondary trigger:** rotation gated by minimum translation

* **Adaptive thresholding:** based on motion noise statistics

This avoids false positives caused by:

* IMU noise

* small ICP corrections

* stationary jitter

* * *

Parameters
----------

### Core Parameters

| Parameter        | Type   | Description                 |
| ---------------- | ------ | --------------------------- |
| `voxel_leaf`     | double | Downsampling leaf size      |
| `correspondence` | double | ICP correspondence distance |
| `epsilon`        | double | ICP convergence threshold   |
| `icp_iterations` | int    | Maximum ICP iterations      |

* * *

### Map Parameters

| Parameter               | Type   | Description                                |
| ----------------------- | ------ | ------------------------------------------ |
| `local_map_max_size`    | int    | Maximum number of points in local map      |
| `scan_trim_size`        | int    | Number of scans retained in sliding window |
| `local_submap_distance` | double | Crop radius (if enabled)                   |

* * *

### Keyframe Parameters

| Parameter                               | Type   | Description                                       |
| --------------------------------------- | ------ | ------------------------------------------------- |
| `keyframe_translation_thresh`           | double | Translation threshold (meters)                    |
| `keyframe_rotation_thresh`              | double | Rotation threshold (radians)                      |
| `keyframe_min_translation_for_rotation` | double | Minimum translation required to consider rotation |

* * *

### Other Parameters

| Parameter            | Type   | Description                               |
| -------------------- | ------ | ----------------------------------------- |
| `EnableCropDistance` | bool   | Enable cropping for ICP input             |
| `fitness_score`      | double | ICP fitness threshold                     |
| `max_noiseDistance`  | double | Max noise threshold for motion estimation |
| `watchdog_timeout`   | int    | Timeout for incoming data                 |

* * *

Local Map Behavior
------------------

* Maintains a **sliding window of recent scans**

* Limits computational cost of ICP

* Periodically trims and rebuilds based on queue

* Optionally cropped to improve performance

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

* Global correction is handled by a downstream SLAM node

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

* Assumes rotation correction is handled upstream

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

* clean separation of concerns

* extensibility toward full SLAM

* This utilizes data pulished from the l2lidar_node V0.2.2

* * *

## Version

V0.5.0    2026-04-05    Initial implelementation RC1


