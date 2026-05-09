//--------------------------------------------------------
//
//  lidar_odometry_node
//  Author: Mark Stegall
//  Module: lidar_odom_imu.cpp
//
//	Purpose:
//		The lidar_odometry_node app is a ROS2 package which provides
//      a simple ROS2 publishing node for odometry derived from the
//      The L2 LiDAR unit by Unitree.
//
//  Goal: Learn about odometry using the L2 methodology.
//        Undertsand and indentify issues with the L2 LiDAR
//          that may limit useful derivation of odometry.
//
//	Implementation
//      This is a simple odometry node for the Unitree L2 4d LiDAR.
//		It subscribes to the l2lidar_node for point clod and IMU data.
//
//      This modlue implements a publisher node for derived odometry from the L2.
//      This has been separated from the initial L2 IMU and point cloud publisher
//      stream to that it could potentially be indepedent of the Lidar sensor and IMU
//      being used.
//
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//		Initial target hardware is RPI5 (ARM64) and an x86_64
//
//
//  ******  TOPIC LISTS *****
//
// --- l2lidar_node publishers ---
//
//  IMU data
//      topic /imu/data
//          frame_id: l2lidar_imu
//
//  Static transforms
//      topic //tf_static
//          frame_id: base_link
//          child_frame_id: l2lidar_frame
//
//          frame_id: l2lidar_frame
//          child_frame_id: l2lidar_imu
//
//
//      The initial pose for the local self determined map is base_linK.
//      The initial map is composed on 'n' scans in this pose.
//      So the initial odom frame is the same as the base_link frame.
//
//		V0.1.0	2026-03-12	Initial package skeleton
//      V0.2.0  2026-03-13  Properly read static TFs l2lidar_node publisher
//                          to align to robot origin
//		V0.3.0	2026-03-18	Rework scan queue trim, diagonostic publihing
//							IMU read back, no processing
//                          Cleaned up baseline with verified reference frames
//      V0.3.1  2026-03-25  Added map frame and TFs for it.
//                          Added below horizon limit on crop box
//      V0.4.0  2026-03-29  Changed the reference frames to be aligned to
//                          to typical ROS2 topics and IDs
//                          Refactored to use more helper methods for publishing
//                          so that code is easier to follow.
//      V0.6.0  2026-05-04  Currently IMU data is not being used but next version
//                          is planned to use IMU data to help confirm stationary
//                          platform state.
//      V0.6.2  2026-05-09  Corrected major bugs involing variable initialization, deltameasured_
//                          was initialzed on allocation
//                          and shadow allocation allocation of 'guess' when stationary logic added
//                          meant guess was not initialized if not stationary
//                          Spelling correction to comments
//                          Added guard checks and mutexes to various calculations and calls
//
//      QtCreator IDE was used in the development
//      This package has NO Qt depdendencies or libraries
//      It uses CMakeList.txt for configuration
//
#include "lidar_odometry_node.h"

//--------------------------------------------------------
//  imuCallback
//  This is used to determine if the platform
//  is stationary using IMU data.
//--------------------------------------------------------
void lidar_odometry_node::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Reset watchdog timer
    {
        std::lock_guard<std::mutex> lock(watchdog_mutex_);
        last_msg_time_ = this->get_clock()->now();
    }

    // TFs need initialization before receiving messages
    if(!static_tf_ready_)
        return;

    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);

    Eigen::Vector3d accel(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);

    rclcpp::Time stamp(msg->header.stamp);

    std::lock_guard<std::mutex> lock(imu_mutex_);

    if (imu_.count == 0) {
        imu_.first_stamp = stamp;
    }

    imu_.last_stamp = stamp;

    imu_.gyro_sum += gyro;

    imu_.accel_sum += accel;

    imu_.accel_sq_sum +=
        accel.cwiseProduct(accel);

    imu_.count++;
}

//--------------------------------------------------------
//  computeIMUstationary
//--------------------------------------------------------
StationaryState lidar_odometry_node::computeIMUstationary()
{
    ImuAccumulator local;

    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        if (imu_.count == 0)
            return StationaryState::Unknown;

        local = imu_;                 // copy
        imu_ = ImuAccumulator();      // reset shared state
    }

    // --- compute outside lock ---
    double duration = (local.last_stamp - local.first_stamp).seconds();

    if (duration <= 0.0)
        return StationaryState::Unknown;

    if (local.count < min_imu_samples_)
        return StationaryState::Unknown;

    // Mean gyro

    Eigen::Vector3d mean_gyro = local.gyro_sum / local.count;
    double gyro_norm = mean_gyro.norm();

    // Accel variance

    Eigen::Vector3d mean_accel = local.accel_sum / local.count;
    Eigen::Vector3d mean_accel_sq = local.accel_sq_sum / local.count;
    Eigen::Vector3d accel_var = mean_accel_sq - mean_accel.cwiseProduct(mean_accel);

    // Numerical protection

    accel_var = accel_var.cwiseMax(0.0);
    double accel_std = std::sqrt(accel_var.sum());

    // Thresholds
    bool gyro_stationary = (gyro_norm < gyro_threshold_);
    bool accel_stationary = (accel_std < accel_std_threshold_);

    if (gyro_stationary && accel_stationary) {
        return StationaryState::Stationary;
    }

    return StationaryState::Moving;
}
