# LiDAR Odometry Processing Flow

This document describes the runtime processing flow of the `lidar_odometry_node`.

The node performs:

- LiDAR scan registration
- IMU-assisted stationary detection
- ICP-based motion estimation
- Rolling submap management
- Keyframe extraction
- TF and odometry publication

---

# High-Level Runtime Architecture

```text
                        +----------------------+
                        |      /imu/data       |
                        +----------+-----------+
                                   |
                                   v
                          IMU Callback Thread
                                   |
                                   v
                        IMU Motion Classification
                                   |
                                   |
+----------------------+           |
|       /points        |           |
+----------+-----------+           |
           |                       |
           v                       |
     Point Cloud Callback          |
           |                       |
           +-----------+-----------+
                       |
                       v
               Point Cloud Cleanup
                       |
                       v
                  TF Validation
                       |
                       v
           lidar_frame -> base_link
                       |
                       v
               Startup Anchor Map
                       |
                       v
               Motion Prediction
                       |
                       v
                ICP Target Build
                       |
                       v
                 ICP Alignment
                       |
                       v
             Alignment Validation
                       |
                       v
            Stationary Suppression
                       |
                       v
               Pose State Update
                       |
         +-------------+-------------+
         |                           |
         v                           v
     Submap Update             Keyframe Logic
         |                           |
         +-------------+-------------+
                       |
                       v
              TF / Odom Publish
                       |
                       v
              Diagnostic Publishers
```

---

# Threading Model

The node uses:

```cpp
rclcpp::executors::MultiThreadedExecutor
```

with two worker threads.

---

# Callback Groups

## IMU Callback Group

Processes:

- IMU statistics
- Gyroscope analysis
- Accelerometer variance
- Stationary classification

---

## Cloud Callback Group

Processes:

- Point cloud cleanup
- ICP alignment
- Pose updates
- Submaps
- Publishing

---

# Startup Flow

At startup the node does NOT immediately perform odometry.

Instead, it accumulates multiple static scans to build an anchor map.

---

# Startup Sequence

```text
Node Startup
     ↓
Initialize ROS interfaces
     ↓
Wait for static TFs
     ↓
Receive point clouds
     ↓
Transform lidar → base_link
     ↓
Accumulate static scans
     ↓
Build immutable anchor map
     ↓
Initialize active submap
     ↓
Begin ICP odometry
```

---

# Initial Anchor Map

The startup anchor map is:

```cpp
anchor_submap_
```

This map:

- remains immutable
- provides stable ICP initialization
- prevents startup drift
- improves local registration stability

---

# Runtime Point Cloud Processing

Each incoming point cloud executes the following pipeline.

---

# Step 1 — Watchdog Reset

```cpp
last_msg_time_ = now();
```

Purpose:

- confirms sensor activity
- prevents watchdog shutdown

---

# Step 2 — Static TF Validation

Before processing, the node verifies:

```text
base_link ↔ lidar
base_link ↔ imu
```

are valid.

If unavailable:

```text
skip frame
retry TF acquisition
```

---

# Step 3 — ROS → PCL Conversion

Incoming ROS cloud:

```cpp
sensor_msgs::msg::PointCloud2
```

is converted into:

```cpp
pcl::PointCloud<pcl::PointXYZI>
```

---

# Step 4 — NaN Filtering

Invalid points are removed:

```cpp
pcl::removeNaNFromPointCloud(...)
```

This prevents:

- ICP instability
- numerical failures
- invalid transforms

---

# Step 5 — Transform Into `base_link`

The LiDAR scan is transformed:

```text
lidar_frame → base_link
```

Result:

```cpp
base_link_scan_
```

This standardizes all registration into robot body coordinates.

---

# Step 6 — Startup Map Initialization

Before odometry begins:

```cpp
*local_map_ += *base_link_scan_;
```

Multiple static scans accumulate into:

```cpp
local_map_
```

Once:

```cpp
init_scans_total >= init_scans_
```

the node:

- freezes the anchor map
- initializes active submap tracking
- enables ICP odometry

---

# Runtime ICP Processing

After initialization, every incoming scan executes ICP registration.

---

# Step 7 — Motion Prediction

The node predicts robot motion using the previous ICP delta.

---

## Prediction Model

```cpp
guess = lastAlignedTF_ * delta_pred;
```

Assumptions:

- approximately constant velocity
- approximately constant direction

---

# Step 8 — Prediction Clamping

The predicted transform is bounded.

---

## Translation Clamp

```cpp
max_predict_dist_
```

Prevents excessive jumps.

---

## Rotation Clamp

```cpp
max_rotation_step_
```

Prevents unstable rotational guesses.

---

# Step 9 — Stationary Detection

The node evaluates whether the robot is stationary.

This uses:

- ICP translation statistics
- IMU motion analysis

---

# ICP Motion Statistics

Running statistics are computed from frame-to-frame motion:

| Variable         | Purpose         |
| ---------------- | --------------- |
| `avgDistance_`   | Mean motion     |
| `sigmaDistance_` | Motion variance |

---

# IMU Classification

The IMU classifier produces:

| State        | Meaning                 |
| ------------ | ----------------------- |
| `Moving`     | Robot moving            |
| `Stationary` | Robot stationary        |
| `Unknown`    | Insufficient confidence |

---

# Combined Stationary Decision

Motion suppression occurs when:

```text
ICP indicates stationary
AND
IMU does not indicate moving
```

This suppresses:

- stationary drift
- ICP noise walk
- false motion accumulation

---

# Step 10 — ICP Target Construction

The node constructs a local registration target from rolling submaps.

---

# Submap Selection

Nearby submaps are selected by:

```text
distance from current pose
```

within:

```cpp
submap_radius_
```

---

# ICP Target Sources

The target cloud may include:

- nearby rolling submaps
- immutable startup anchor map

---

# Step 11 — ICP Registration

The node performs ICP alignment.

---

# ICP Inputs

| Input  | Purpose              |
| ------ | -------------------- |
| Source | Current LiDAR scan   |
| Target | Local merged submaps |
| Guess  | Predicted transform  |

---

# ICP Solve

```cpp
icp.align(aligned, guess);
```

Result:

```cpp
T_alignment
```

---

# Step 12 — ICP Validation

The ICP solution is validated before use.

---

# Convergence Check

```cpp
icp.hasConverged()
```

Rejects failed solves.

---

# Fitness Validation

```cpp
icp.getFitnessScore()
```

Rejects poor alignments.

---

# Step 13 — Delta Motion Computation

Frame-to-frame motion is computed:

```cpp
deltaMeasured_ =
    lastAlignedTF_.inverse() * T_alignment;
```

This becomes the next prediction basis.

---

# Step 14 — Stationary Motion Suppression

If stationary:

```text
freeze pose
zero motion delta
suppress drift
```

Otherwise:

```text
update transform state
update prediction state
```

---

# Step 15 — Pose State Update

The active robot pose becomes:

```cpp
T_odom_base_
```

representing:

```text
odom → base_link
```

---

# Step 16 — Active Submap Accumulation

If moving:

```cpp
*active_submap_ += aligned;
```

The aligned scan is inserted into the active rolling submap.

---

# Step 17 — Submap Closure Logic

The current submap closes when:

| Condition            | Threshold             |
| -------------------- | --------------------- |
| Translation distance | `submap_dist_thresh_` |
| Rotation change      | `submap_yaw_thresh_`  |
| Point count          | `submap_max_points_`  |

---

# Submap Closure Sequence

```text
Close active submap
      ↓
Compute centroid
      ↓
Push into rolling submap list
      ↓
Discard oldest if necessary
      ↓
Start new active submap
```

---

# Step 18 — Keyframe Evaluation

The node evaluates whether a new keyframe should be created.

---

# Keyframe Triggers

| Trigger     | Description           |
| ----------- | --------------------- |
| Translation | Robot moved enough    |
| Rotation    | Robot rotated enough  |
| Time        | Too much time elapsed |

---

# Keyframe Generation

When triggered:

- full-resolution scan is used
- scan is transformed into odom frame
- pose is published
- cloud is published

---

# Step 19 — TF Publication

The node publishes:

```text
odom → base_link
```

for the ROS TF tree.

---

# Step 20 — Path Publication

A diagnostic path message is updated and published.

Purpose:

- RViz visualization
- drift observation
- trajectory debugging

---

# Step 21 — Odometry Publication

The node publishes:

```cpp
nav_msgs::msg::Odometry
```

containing:

- pose
- orientation
- timestamps
- frame IDs

---

# Step 22 — Diagnostic Cloud Publication

The aligned ICP scan is published for RViz visualization.

---

# Watchdog Flow

A periodic watchdog timer validates sensor activity.

---

# Watchdog Sequence

```text
Timer tick
    ↓
Compute elapsed time since last message
    ↓
Compare against watchdog timeout
    ↓
If exceeded:
    ↓
Log error
    ↓
Shutdown ROS node
```

---

# Coordinate Frame Flow

```text
LiDAR Scan
    ↓
lidar_frame
    ↓
Transform
    ↓
base_link
    ↓
ICP Alignment
    ↓
odom
```

---

# ICP Data Flow

```text
Current Scan
      ↓
Transform to base_link
      ↓
Predict pose
      ↓
Build ICP target
      ↓
Run ICP
      ↓
Validate result
      ↓
Update odometry pose
```

---

# Submap Lifecycle

```text
Create active submap
        ↓
Accumulate aligned scans
        ↓
Motion threshold exceeded
        ↓
Close submap
        ↓
Insert into rolling list
        ↓
Create new active submap
```

---

# Keyframe Lifecycle

```text
Motion detected
      ↓
Threshold exceeded
      ↓
Create keyframe
      ↓
Publish pose
      ↓
Publish cloud
```

---

# Runtime States

| State           | Description                |
| --------------- | -------------------------- |
| Startup         | Building anchor map        |
| Active Tracking | ICP odometry running       |
| Stationary      | Motion suppression enabled |
| Recovery        | ICP rejection / retry      |
| Shutdown        | Watchdog timeout           |

---

# Failure Handling

The node rejects frames when:

- ICP fails
- fitness score too high
- TF unavailable
- cloud becomes empty
- NaN corruption detected

---

# Overall Design Goals

The architecture is optimized for:

- real-time operation
- long-duration runtime stability
- reduced stationary drift
- scalable local mapping
- future SLAM integration
- robust sensor fault handling

---

# Summary

The `lidar_odometry_node` processing pipeline combines:

- LiDAR scan registration
- IMU-assisted motion classification
- rolling submap management
- ICP prediction and validation
- keyframe extraction
- ROS TF integration

to produce a robust real-time LiDAR odometry backend suitable for long-duration robotic operation.
