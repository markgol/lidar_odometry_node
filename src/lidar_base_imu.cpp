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
//
//      QtCreator IDE was used in the development
//      This package has NO Qt depdendencies or libraries
//      It uses CMakeList.txt for configuration
//
#include "lidar_odometry_node.h"

//--------------------------------------------------------
//  imu callback
//  This accumulates a queue of IMU packets
//  This is here as placeholder for future expansion
//  using the IMU to better estimate the pose position
//  and provide improved deskewing
//--------------------------------------------------------
void lidar_odometry_node::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Reset watchdog timer
    last_msg_time_ = this->get_clock()->now();

    // TFs need initialization before receiving messages
    if(!static_tf_ready_)
        return;

    std::lock_guard<std::mutex> lock(imu_mutex_);

    // Keep only messages since last LiDAR frame
    if (!last_scan_time_.nanoseconds()) {
        last_scan_time_ = msg->header.stamp;
    }
    long long msgTime = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
    if (msgTime > last_scan_time_.nanoseconds()) {
        imu_queue_.push_back(msg);
        if(imu_queue_.size()>max_imu_queue_) {
            imu_queue_.pop_front();
        }
    }
}

//--------------------------------------------------------
//  integrateImuRotation
//  *** This is just an example and is not actually being used ***
//
//  This integrates a queue of IMU packets
//  This will likely never get used in its current form
//  It is being kept as examples of some usefull conversion
//  syntax and transform examples.
//
//  Note:
//--------------------------------------------------------
Eigen::Matrix3f lidar_odometry_node::integrateImuRotation()
{
    if (imu_queue_.size() < 2)
        return Eigen::Matrix3f::Identity();

    Eigen::Matrix3f R_total = Eigen::Matrix3f::Identity();

    for (size_t i = 1; i < imu_queue_.size(); ++i)
    {
        const auto &imu_prev = imu_queue_[i-1];
        const auto &imu_curr = imu_queue_[i];

        // Compute dt in seconds using nanoseconds
        long long curr_nsec =
            (long long)imu_curr->header.stamp.sec * 1000000000LL +
            imu_curr->header.stamp.nanosec;

        long long prev_nsec =
            (long long)imu_prev->header.stamp.sec * 1000000000LL +
            imu_prev->header.stamp.nanosec;

        double dt = (curr_nsec - prev_nsec) * 1e-9;

        if (dt <= 0.0 || dt > 0.05)
            continue;

        // Angular velocity in IMU frame (rad/sec)
        Eigen::Vector3f omega_imu(imu_curr->angular_velocity.x,
                                  imu_curr->angular_velocity.y,
                                  imu_curr->angular_velocity.z);

        // convert to base_link frame
        Eigen::Vector3f omega_base = R_base_imu_ * omega_imu;

        // delta time correction
        Eigen::Vector3f theta = omega_base * dt;

        float angle = theta.norm();

        Eigen::Matrix3f R_delta = Eigen::Matrix3f::Identity();

        if (angle > 1e-8f)
        {
            Eigen::Vector3f axis = theta.normalized();
            R_delta = Eigen::AngleAxisf(angle, axis).toRotationMatrix();
        }

        // Accumulate total rotation
        R_total =  R_delta * R_total;
    }

    // Clear queue after integration
    imu_queue_.clear();

    return R_total;
}

