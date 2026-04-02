//--------------------------------------------------------
//
//  lidar_odometry_node
//  Author: Mark Stegall
//  Module: odom_TFs_publishers.cpp
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
//      This module implements a publisher node for derived odometry from the L2.
//      This has been separated from the initial L2 IMU and point cloud publisher
//      stream to that it could potentially be indepedent of the Lidar sensor and IMU
//      being used.
//
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//		Initial target hardware is RPI5 (ARM64) and an x86_64
//
//      TRANSFORM TREE
//
//      map(relative world world with origin at starting point of robot)
//        └──  odom (local robot position in self determined sliding local_map based on ICP alignment)
//              └── base_link (origin of the robot coordinate system)
//                   └── l2lidar_frame (lidar sensor from l2lidar_node)
//                         └── l2lidar_imu (imu sensor from l2lidar_node)
//
//
//  ******  TOPIC LISTS *****
//
// --- lidar_odometry_node publishers ---
//
//      topic /aligned_scan
//          frame id: base_link
//
//      topic /path
//          frame_id: odom
//
//      topic /odom
//          frame_id: odom
//          child_frame_id: base_link
//
//      topic /submap_crop_marker
//          frame_id: odom
//          ns: submap_crop
//          id: 0
//          type: 1
//
//  Dynamic transforms published by lidar_odometry_node
//      topic /tf
//          frame_id: odom
//          child_frame_id: base_link
//          transform:
//
// --- l2lidar_node publishers ---
//
//  IMU data
//      topic /imu/data
//          frame_id: l2lidar_imu
//
//  Point cloud data
//      topic /points
//          frame_id: l2lidar_frame
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
//  publishOdometry
//--------------------------------------------------------
void lidar_odometry_node::publishOdometry(const rclcpp::Time &stamp)
{
    //------------------------------------------
    // publish Odom->lidar
    //------------------------------------------
    Eigen::Matrix3f R = T_odom_base_.block<3,3>(0,0);
    Eigen::Vector3f t = T_odom_base_.block<3,1>(0,3);
    Eigen::Quaternionf q(R);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odometry_frame_id_;
    odom.child_frame_id = robot_frame_id_;

    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);
}

//--------------------------------------------------------
// publishTransform()
// publish a transform
//--------------------------------------------------------
void lidar_odometry_node::publishTransform(Eigen::Matrix4f Transform,
                      std::string frame_id,
                      std::string child_frame_id,
                      const rclcpp::Time &stamp)
{
    geometry_msgs::msg::TransformStamped tf2send;

    tf2send.header.stamp = stamp;
    tf2send.header.frame_id = frame_id;
    tf2send.child_frame_id = child_frame_id;

    Eigen::Matrix3f Rmo = Transform.block<3,3>(0,0);
    Eigen::Vector3f tmo = Transform.block<3,1>(0,3);
    Eigen::Quaternionf qmo(Rmo);

    tf2send.transform.translation.x = tmo.x();
    tf2send.transform.translation.y = tmo.y();
    tf2send.transform.translation.z = tmo.z();

    tf2send.transform.rotation.x = qmo.x();
    tf2send.transform.rotation.y = qmo.y();
    tf2send.transform.rotation.z = qmo.z();
    tf2send.transform.rotation.w = qmo.w();

    tf_broadcaster_->sendTransform(tf2send);
}

//--------------------------------------------------------
//
//  publishPath()
//
//  This publishes a path message
//
//--------------------------------------------------------
void lidar_odometry_node::publishPath(Eigen::Matrix4f Transform,
                                      std::string frame_id,
                                      const rclcpp::Time &stamp)
{
    geometry_msgs::msg::PoseStamped path;

    path.header.stamp = stamp;
    path.header.frame_id = frame_id;

    path.pose.position.x = Transform(0,3);
    path.pose.position.y = Transform(1,3);
    path.pose.position.z = Transform(2,3);

    Eigen::Matrix3f RP = Transform.block<3,3>(0,0);
    Eigen::Quaternionf qp(RP);

    path.pose.orientation.x = qp.x();
    path.pose.orientation.y = qp.y();
    path.pose.orientation.z = qp.z();
    path.pose.orientation.w = qp.w();

    // *** this needs verification ***
    // Is separate publisher needed for multiple paths?
    path_msg_.header.frame_id = frame_id;
    path_msg_.poses.push_back(path);
    path_msg_.header.stamp = stamp;

    path_pub_->publish(path_msg_);
    return;
}

//--------------------------------------------------------
//  initStaticTF
//  This is used to acquire the static TFs needed that
//  are published by the l2lidar_node
//
//  base_link  is the robor body
//  The transforms from published TFs from the l2lidar_node
//      base_link is the robot frame reference
//      lidar_frame is the lidar scan frame reference
//      imu_frame is the imu sensor frame reference
//
//      T_base_lidar_ = base_link -> lidar_frame
//      T_lidar_imu_ = lidar_frame -> imu_frame
//
//  Calculated TF
//      T_base_imu = T_base_lidar * T_lidar_imu
//      rotation only transform
//      R_base_imu = R_base_lidar * R_lidar_imu
//
//--------------------------------------------------------
void lidar_odometry_node::initStaticTF()
{
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
            try
            {
                // base_link -> l2lidar frame
                auto tf_base_msg = tf_buffer_->lookupTransform(
                    "base_link",
                    "l2lidar_frame",
                    rclcpp::Time(0));

                Eigen::Isometry3d T1 =
                    tf2::transformToEigen(tf_base_msg.transform);

                T_base_lidar_ = T1.matrix().cast<float>();

                // lidar frame -> imu frame
                auto tf_imu_msg =
                    tf_buffer_->lookupTransform(
                        "l2lidar_frame",
                        "l2lidar_imu",
                        rclcpp::Time(0));

                Eigen::Isometry3d T2 =
                    tf2::transformToEigen(tf_imu_msg.transform);

                T_lidar_imu_ = T2.matrix().cast<float>();

                // Precompute rotation IMU → base_link
                Eigen::Matrix3f R_base_lidar =
                    T_base_lidar_.block<3,3>(0,0);

                Eigen::Matrix3f R_lidar_imu =
                    T_lidar_imu_.block<3,3>(0,0);

                R_base_imu_ = R_base_lidar * R_lidar_imu;

                T_base_imu_ =  T_base_lidar_ * T_lidar_imu_;

                // done, mark as ready
                static_tf_ready_ = true;

                RCLCPP_INFO(get_logger(),
                            "Static TF acquired: base_link -> l2lidar_frame -> l2lidar_imu");

                tf_timer_->cancel();
            }
            catch (tf2::TransformException &ex)
            {
                tf_retry_count_++;

                RCLCPP_WARN(get_logger(),
                            "Waiting for static TF (%d/%d)",
                            tf_retry_count_,
                            tf_max_retries_);

                if (tf_retry_count_ >= tf_max_retries_)
                {
                    RCLCPP_FATAL(get_logger(),
                                 "Static TF never received: %s",
                                 ex.what());

                    rclcpp::shutdown();
                }
            }
        }
        );
}

//--------------------------------------------------------
//  TFPointCloud
//  (
//      pointer to point cloud that is to be transformed,
//      Transform to be applied
//  )
//  returns ptr to transformed point cloud
//--------------------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_odometry_node::TFPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const Eigen::Matrix4f &T)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud, *transformed, T);
    return transformed;
}

//--------------------------------------------------------
//  calculateDistanceStats
//  Calculate mean, variance for Value
//  Alpha is the time constant, 0.0 < Alpha <= 1.0
//--------------------------------------------------------
void lidar_odometry_node::calculateStats(
    double Value,
    double* MeanValue, double* sigmaValue,
    double Alpha
    )
{
    double delta = Value - *MeanValue;

    // Update mean (EWMA)
    *MeanValue += Alpha * delta;

    // Update variance (EWMA)
    *sigmaValue = (1.0 - Alpha) * (*sigmaValue)
                  + Alpha * delta * (Value - (*MeanValue));

}

//--------------------------------------------------------
//  DistanceCrop
//  return a point cloud that is cropped by a max distance
//--------------------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_odometry_node::CropDistance(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    double MaxDistance)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);

    // Precompute squared distance to avoid sqrt (faster)
    const double max_dist_sq = MaxDistance * MaxDistance;

    cropped->reserve(cloud->points.size());

    for (const auto& pt : cloud->points)
    {
        double dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

        if (dist_sq > max_dist_sq)
            continue;

        cropped->points.push_back(pt);
    }

    // Preserve metadata
    cropped->width  = static_cast<uint32_t>(cropped->points.size());
    cropped->height = 1;
    cropped->is_dense = cloud->is_dense;

    return cropped;
}
