### Updated 2025-05-04 V0.6.0

lidar_odometry_node
===================

This represents a major architectrual change from V0.5.1.  This just initial README.md which needs  updatin to be complete.

Overview
===================

`lidar_odometry_node` performs real-time LiDAR-based odometry using ICP alignment against a **locally constructed map representation**. The system is designed for robustness on ground-based robots operating in uneven terrain, without relying on IMU-based pose correction.

The node produces:

* `odom → base_link` transform

* Odometry messages

* Keyframes for downstream SLAM (e.g., pose graph backend)

* * *

Key Design Principles
---------------------

### 1. Absolute Pose Estimation (Not Incremental Drift Integration)

ICP is used as an **absolute pose estimator** relative to a local map (anchor + submaps), not as a pure frame-to-frame motion estimator.

This avoids long-term drift accumulation typical of incremental odometry pipelines.

* * *

### 2. Anchor + Submap Architecture

#### Anchor Submap

* Built once at startup while the robot is stationary

* Immutable

* Provides a **stable global reference**

* Prevents long-term drift

#### Active Submap

* Accumulates recent aligned scans

* Represents current local geometry

* Periodically closed and converted into a frozen submap

#### Frozen Submaps

* Stored in a bounded queue

* Provide additional spatial constraints during motion

* Improve ICP robustness during revisits or low-feature regions

* * *

### 3. ICP Target Construction

Each ICP iteration aligns the incoming scan against a target composed of:

* Anchor submap (always or conditionally included)

* Nearby frozen submaps

* Optionally the active submap

This ensures:

* Strong constraints near origin

* Good local alignment during motion

* Reduced degeneracy compared to a sliding window map

* * *

### 4. Motion Model (Prediction)

A constant-velocity SE(3) motion model is used:
    T_pred = T_last * ΔT_previous

This improves:

* ICP convergence speed

* Stability under sparse geometry

* Rotation estimation

* * *

### 5. Stationary Detection and Motion Suppression

ICP becomes underconstrained when the robot is stationary. To prevent drift:

#### Detection

Based on **measured motion statistics**:

* Mean translation

* Standard deviation

* (optionally) rotation magnitude

#### Handling

* Motion is **suppressed (not integrated)** when stationary

* ICP still runs for alignment, but its motion output is ignored

Key concept:

> Measured motion is preserved, but applied motion is filtered.

* * *

### 6. Separation of Motion Signals

Two motion representations are maintained:

* `deltaMeasured_`: raw motion from ICP (always updated)

* `deltaApplied_`: filtered motion used for state updates

This ensures:

* Reliable stationary detection

* Immediate response when motion resumes

* * *

### 7. Motion Clamping (Stability Control)

To prevent instability from ICP outliers:

* Translation is bounded per frame

* Rotation is bounded per frame

Clamping is applied **before updating the pose state**

* * *

### 8. Submap Management

Submaps are closed using multiple criteria:

* Pose change (translation or rotation)

* Maximum size threshold

* Stationary overflow protection

During stationary periods:

* Active submap growth is suppressed

* * *

Processing Pipeline
-------------------

### 1. Input

* `sensor_msgs::PointCloud2` from LiDAR

* Transform: `lidar → base_link`

* * *

### 2. Initialization Phase

* Accumulate scans while stationary

* Build anchor submap

* Initialize transforms to identity

* * *

### 3. Per-Scan Processing

1. Transform scan → `base_link`

2. Predict pose using motion model

3. Build ICP target (anchor + submaps)

4. Run ICP alignment

5. Compute measured motion (`deltaMeasured_`)

6. Detect stationary condition

7. Apply motion filtering → `deltaApplied_`

8. Clamp motion (translation + rotation)

9. Update pose (`T_odom_base_`)

10. Accumulate into active submap (if moving)

11. Close submap if needed

12. Publish:
* Transform

* Odometry

* Path

* Keyframes

* * *

Frames
------

* `odom`: Local reference frame (drift-limited)

* `base_link`: Robot frame

* `lidar`: Sensor frame (static transform)

* * *

Assumptions
-----------

* Robot starts stationary long enough to build anchor map

* Ground-based platform

* Roll/pitch may vary (terrain), yaw unconstrained

* No reliance on IMU for pose correction

* * *

Advantages
----------

* Resistant to long-term drift

* Robust to IMU noise or failure

* Handles stationary degeneracy explicitly

* Supports rotation-in-place (tracked robots)

* Stable ICP convergence via motion prediction

* * *

Limitations
-----------

* Requires good initial stationary scan set

* ICP degeneracy still possible in extremely sparse environments

* No global consistency without external SLAM backend

* * *

Integration with SLAM
---------------------

This node provides **high-quality local odometry** for:

* Pose graph SLAM

* Loop closure systems

* Map optimization (e.g., Ceres-based backends)

It does not perform:

* Loop closure

* Global map optimization

* * *

Future Improvements
-------------------

* Yaw-specific damping (preserve roll/pitch freedom)

* Inclusion of IMU data in stationary determinination

* Adaptive target selection radius (only if needed to reduce compuation time)

* Integration of wheel odometry (optional)

* * *

Summary
-------

This implementation transitions from a traditional sliding-map ICP approach to a **structured, stability-focused odometry system**:

* Anchor map for long-term stability

* Submaps for local consistency

* Absolute pose estimation via ICP

* Explicit handling of stationary degeneracy

The result is significantly improved robustness over long durations, especially in environments where IMU data is unreliable.

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



V0.6.0    2026-05-04    This is major architectural change thats uses immutable submaps for ICP matching along with detection for stationary state.  This is to reduce the degeneracy when the platform is not moving and noise results in drift.
