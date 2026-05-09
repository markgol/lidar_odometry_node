## Updated 2025-05-09 V0.6.2

# lidar_odometry_node

A ROS 2 LiDAR odometry backend implementing real-time scan matching using ICP (Iterative Closest Point), rolling submaps, IMU-assisted stationary detection, and multi-threaded execution.

The node estimates robot motion from LiDAR point clouds while minimizing drift during stationary operation.

---

# Features

- Real-time LiDAR odometry
- ICP scan registration
- Rolling submap architecture
- Immutable startup anchor map
- IMU-assisted stationary detection
- Motion prediction and clamping
- Keyframe extraction
- TF publishing
- Odometry publishing
- Diagnostic RViz outputs
- Watchdog failure detection
- Multi-threaded ROS 2 execution

---

# System Architecture

```text
                 +-------------------+
                 |   /imu/data       |
                 +---------+---------+
                           |
                           v
                    IMU Callback
                           |
                           v
                 Stationary Detection
                           |
                           |
+-------------------+      |
|     /points       |      |
+---------+---------+      |
          |                |
          v                |
    Cloud Callback         |
          |                |
          +----------------+
                   |
                   v
           Motion Prediction
                   |
                   v
             ICP Alignment
                   |
                   v
           Pose Estimation
                   |
          +--------+--------+
          |                 |
          v                 v
     Submap Update     Keyframe Logic
          |                 |
          +--------+--------+
                   |
                   v
        TF / Odom / Path Publish
```

---

# Main Entry Point

```cpp
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<lidar_odometry_node>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);

    executor.add_node(node);
    executor.spin();

    executor.remove_node(node);

    rclcpp::shutdown();

    return 0;
}
```

---

# Multi-Threaded Execution

The node uses:

```cpp
rclcpp::executors::MultiThreadedExecutor
```

with two execution threads.

## Purpose

Two callback groups are created:

- IMU callback group
- Point cloud callback group

This allows:

- Concurrent IMU processing
- Concurrent point cloud processing
- Reduced ICP latency
- Improved throughput

## Callback Threading

| Thread   | Responsibility             |
| -------- | -------------------------- |
| Thread 1 | IMU callbacks              |
| Thread 2 | Point cloud ICP processing |

---

# Watchdog Timer

The watchdog timer shuts down the node if sensor data stops arriving.

## Failure Cases

- LiDAR powered off
- Upstream node crash
- ROS topic failure
- Sensor communication timeout

## Logic

```cpp
void lidar_odometry_node::watchdogCheck()
```

If no messages are received within:

```yaml
watchdog_timeout: 60
```

the node logs an error and shuts down ROS.

---

# Coordinate Frames

The system operates using the following frames:

| Frame       | Description          |
| ----------- | -------------------- |
| `odom`      | Local odometry frame |
| `base_link` | Robot body frame     |
| `lidar`     | LiDAR sensor frame   |
| `imu`       | IMU sensor frame     |

---

# Processing Pipeline

```text
Receive Point Cloud
        ↓
Validate TF
        ↓
Transform lidar → base_link
        ↓
Build Initial Anchor Map
        ↓
Predict Motion
        ↓
Build ICP Target
        ↓
ICP Registration
        ↓
Validate Solution
        ↓
Suppress Stationary Drift
        ↓
Update Submaps
        ↓
Generate Keyframes
        ↓
Publish TF / Odom / Path
```

---

# Startup Initialization

At startup the node accumulates multiple static scans before odometry begins.

## Purpose

This creates an immutable startup reference map called:

```cpp
anchor_submap_
```

## Initialization Flow

```text
Static Robot
    ↓
Accumulate scans
    ↓
Create anchor map
    ↓
Begin ICP odometry
```

## Recommended Scan Count

For Unitree L2 LiDAR:

```yaml
init_scans: 20
```

Typical useful range:

- 20–50 scans

---

# Subscribers

## IMU Subscriber

### Topic

```text
/imu/data
```

### Message Type

```cpp
sensor_msgs::msg::Imu
```

### Purpose

Used for:

- Motion classification
- Stationary detection
- Drift suppression logic

---

## Point Cloud Subscriber

### Topic

```text
/points
```

### Message Type

```cpp
sensor_msgs::msg::PointCloud2
```

### Important Requirement

The incoming point cloud MUST NOT be pre-rotated or motion corrected.

ICP requires raw sensor-relative geometry.

---

# Publishers

| Topic                    | Type        | Purpose               |
| ------------------------ | ----------- | --------------------- |
| `/odom`                  | Odometry    | Robot pose            |
| `/aligned_scan`          | PointCloud2 | ICP aligned scan      |
| `/path`                  | Path        | Diagnostic trajectory |
| `/lidar/keyframes/pose`  | PoseStamped | Keyframe poses        |
| `/lidar/keyframes/cloud` | PointCloud2 | Keyframe clouds       |

---

# ICP Registration

The node uses PCL ICP:

```cpp
pcl::IterativeClosestPoint
```

## Configuration

```cpp
icp.setTransformationEpsilon(epsilon_);
icp.setEuclideanFitnessEpsilon(epsilon_);
icp.setMaximumIterations(icp_iterations_);
icp.setMaxCorrespondenceDistance(correspondence_);
```

---

# Motion Prediction

Motion prediction uses the previous frame delta transform.

## Prediction Model

```cpp
guess = lastAlignedTF_ * delta_pred;
```

This assumes approximately:

- Constant velocity
- Constant direction

---

# Stationary Detection

One major source of LiDAR odometry drift is false motion while stationary.

This node suppresses that behavior using:

- ICP noise statistics
- IMU motion analysis

---

## ICP Noise Statistics

Running statistics are computed from ICP translation estimates.

### Metrics

| Metric           | Purpose                |
| ---------------- | ---------------------- |
| `avgDistance_`   | Mean translation noise |
| `sigmaDistance_` | Motion variance        |

---

## IMU Motion States

The IMU logic classifies motion as:

| State        | Meaning                 |
| ------------ | ----------------------- |
| `Moving`     | Robot is moving         |
| `Stationary` | Robot is stationary     |
| `Unknown`    | Insufficient confidence |

---

## Combined Motion Logic

Motion is suppressed only when:

```text
ICP indicates stationary
AND
IMU does not indicate moving
```

This dramatically reduces long-term stationary drift.

---

# ICP Target Construction

The node builds a local ICP target map from nearby rolling submaps.

## Selection Process

1. Compute distance to each submap
2. Sort by nearest distance
3. Merge nearby submaps
4. Optionally include anchor map

---

## ICP Target Parameters

| Parameter              | Purpose                  |
| ---------------------- | ------------------------ |
| `submap_radius`        | Search radius            |
| `max_selected_submaps` | Number of nearby submaps |

---

# Submaps

## Purpose

Submaps provide:

- Local registration stability
- Reduced ICP search complexity
- Improved scalability

---

## Active Submap

Aligned scans accumulate into:

```cpp
active_submap_
```

---

## Submap Closure Conditions

A submap closes when:

| Condition            | Threshold             |
| -------------------- | --------------------- |
| Translation distance | `submap_dist_thresh_` |
| Rotation change      | `submap_yaw_thresh_`  |
| Point count          | `submap_max_points_`  |

---

# Delta Clamping

Large ICP jumps can destabilize tracking.

The node clamps:

- Translation magnitude
- Rotation magnitude

---

## Translation Clamp

```cpp
max_predict_dist_
```

---

## Rotation Clamp

```cpp
max_rotation_step_
```

---

# Keyframe Extraction

Keyframes provide sparse long-term map snapshots.

## Trigger Conditions

Keyframes are created based on:

- Translation distance
- Rotation change
- Time elapsed

---

## Published Keyframe Data

| Topic                    | Description        |
| ------------------------ | ------------------ |
| `/lidar/keyframes/pose`  | Pose of keyframe   |
| `/lidar/keyframes/cloud` | Full aligned cloud |

---

# TF Publishing

The node publishes:

```text
odom → base_link
```

representing the robot pose estimate.

---

# Odometry Publishing

Odometry messages contain:

- Position
- Orientation
- Timestamp
- Frame IDs

---

# Diagnostic Outputs

## Aligned Scan

Publishes the ICP-aligned point cloud for RViz visualization.

---

## Path Visualization

Publishes robot trajectory for debugging.

---

# Parameters

# Core Parameters

| Parameter        | Default | Description                 |
| ---------------- | ------- | --------------------------- |
| `init_scans`     | `20`    | Startup anchor scans        |
| `voxel_leaf`     | `0.03`  | Downsample voxel size       |
| `correspondence` | `1.0`   | ICP correspondence distance |
| `epsilon`        | `1e-6`  | ICP convergence epsilon     |
| `icp_iterations` | `25`    | Max ICP iterations          |
| `fitness_score`  | `0.3`   | Maximum ICP fitness         |

---

# Submap Parameters

| Parameter              | Default  | Description              |
| ---------------------- | -------- | ------------------------ |
| `max_submaps`          | `8`      | Retained rolling submaps |
| `submap_radius`        | `4.0`    | ICP search radius        |
| `max_selected_submaps` | `3`      | Nearby submaps used      |
| `submap_max_points`    | `400000` | Submap closure limit     |

---

# Motion Suppression Parameters

| Parameter             | Default | Description                       |
| --------------------- | ------- | --------------------------------- |
| `max_noiseDistance`   | `0.005` | ICP stationary threshold          |
| `gyro_threshold`      | `0.03`  | IMU gyro stationary threshold     |
| `accel_std_threshold` | `0.30`  | Accelerometer stability threshold |
| `min_imu_samples`     | `20`    | Required IMU sample count         |

---

# Keyframe Parameters

| Parameter                     | Default | Description           |
| ----------------------------- | ------- | --------------------- |
| `keyframe_translation_thresh` | `0.5`   | Translation trigger   |
| `keyframe_rotation_thresh`    | `0.3`   | Rotation trigger      |
| `keyframe_last_frame_time`    | `2.0`   | Max keyframe interval |

---

# Example YAML Configuration

```yaml
lidar_odometry_node:
  ros__parameters:

    init_scans: 20

    voxel_leaf: 0.03
    correspondence: 1.0
    epsilon: 1.0e-6
    icp_iterations: 25
    fitness_score: 0.3

    max_submaps: 8
    submap_radius: 4.0
    max_selected_submaps: 3
    submap_max_points: 400000

    max_predict_dist: 1.5
    max_rotation_step: 0.785398

    gyro_threshold: 0.03
    accel_std_threshold: 0.30
    min_imu_samples: 20

    keyframe_translation_thresh: 0.5
    keyframe_rotation_thresh: 0.3

    watchdog_timeout: 60
```

---

# Runtime Behavior

## Startup

```text
Accumulate anchor scans
→ Build immutable anchor map
→ Start odometry tracking
```

---

## Moving

```text
Predict motion
→ ICP align
→ Update pose
→ Extend active submap
```

---

## Stationary

```text
Detect stationary state
→ Freeze pose estimate
→ Suppress ICP drift
```

---

# RViz Visualization

Useful visualization topics:

| Topic                    | Purpose               |
| ------------------------ | --------------------- |
| `/aligned_scan`          | ICP output            |
| `/path`                  | Robot trajectory      |
| `/tf`                    | Coordinate transforms |
| `/lidar/keyframes/cloud` | Sparse keyframes      |

---

# Performance Notes

## Recommended Hardware

- Multi-core CPU
- Fast memory bandwidth
- Hardware floating point acceleration

---

## ICP Performance

Typical ICP timing:

| Environment        | Typical Time |
| ------------------ | ------------ |
| Small indoor map   | 10–30 ms     |
| Medium environment | 30–80 ms     |
| Large complex map  | 80–150 ms    |

---

# Important Constraints

## Raw Point Clouds Required

The upstream LiDAR driver must NOT pre-correct rotation.

This node expects:

```text
raw sensor-relative scans
```

Applying external pose correction before ICP can corrupt registration.

---

# Future Extensions

Planned extensions may include:

- Global SLAM backend
- Loop closure
- Map optimization
- GPS fusion
- Wheel encoder fusion
- Persistent map serialization
- Multi-resolution ICP
- GPU acceleration

---

# Dependencies

## ROS 2 Packages

- `rclcpp`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`

---

## Third-Party Libraries

- PCL
- Eigen

---

# Build

## CMake

```bash
colcon build --packages-select lidar_odometry
```

---

# Run

```bash
ros2 run lidar_odometry lidar_odometry_node
```

---

# License

GPL V3

---

# Summary

This node implements a robust real-time LiDAR odometry system using:

- ICP scan registration
- Rolling submaps
- IMU-assisted drift suppression
- Motion prediction
- Multi-threaded execution

The architecture is designed for:

- Long-duration operation
- Real-time performance
- Stable stationary behavior
- Future SLAM extensibility

## Author Notes

This node is designed for **modular SLAM system integration**, emphasizing:

* real-time performance

* clean separation of processing concerns

* extensibility toward full SLAM

* This utilizes data pulished from the l2lidar_node V0.2.3

* * *

## Version

**V0.5.0**    2026-04-15

This is the initial public release. It is operable but has not been fully tested. It has further development that is being done. This initial release is part of the projects larger over-all skeleton for SLAM operation. It explcitily does not use IMU or other odometry sensors. It relies purely on ICP SE(3) matching to determine odometry. It has additional noise filtering to detect when the platform is not moving. This was done to reduce drift accumulation when the platform isn't moving. It also has a radius crop of the local map in order to reduce processing requirements. The node assumes that the plaform is stationary when it first starts. This allows the node to create a local map of the platform's location.

**V0.6.0**    2026-05-04

This is major architectural change thats uses immutable submaps for ICP matching along with detection for stationary state. This is to reduce the degeneracy when the platform is not moving and noise results in drift.

**V0.6.1**    2026-05-06

Changed the ros2 main() to use multithreaded executor spin. This allows the IMU and point cloud callbacks to run in indepent threads. This keeps things like ICP from blocking receipt of the IMU data. Also added the use of the IMU data to estimate if the robot is moving, stationary or unknown. This is combined with the ICP results to determine if the robot is stationary.

**V0.6.2**    2026-05-06

Corrected major bugs involing variable initialization, deltameasured_ was initialzed on allocation.
and shadow allocation allocation of 'guess' when stationary logic.
Added mean guess was not initialized if not stationary.
Spelling correction to comments.
Added guard checks and mutexes to various calculations and calls.
s
