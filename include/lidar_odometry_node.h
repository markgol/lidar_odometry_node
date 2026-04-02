//--------------------------------------------------------
//
//  lidar_odometry_node
//  Author: Mark Stegall
//  Module: lidar_odometry_node.h
//
//	Purpose:
//		The lidar_odometry_node app is a ROS2 package which provides
//      a simple ROS2 publishing node for odometry derived from the
//      The L2 LiDAR unit by Unitree.
//
//  Goal: Learn about odometry using the the Untireee 4D LiDAR L2 hardware
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
//		V0.1.0	2026-03-12	Initial package skeleton
//      V0.2.0  2026-03013  Properly read static TFs l2lidar_node publisher
//                          to align to robot origin
//		V0.3.0	2026-03-18	Rework scan queue trim, diagonostic publihing
//							IMU read back, no processing
//                          Cleaned up baseline with verified reference frames
//      V0.3.1  2026-03-19  adding in map frame, path in map frame
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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Dense>

#include <chrono>

//--------------------------------------------------------
//  Class lidar_odometry_node
//--------------------------------------------------------
class lidar_odometry_node : public rclcpp::Node
{
public:

    //constructor
    lidar_odometry_node();
    // no explicit destructor

private:    // private class methods

    // This is called whenever a new point cloud scan is received (5.55Hz for L2)
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // This is called whenever an Imu message is received (~250Hz for L2)
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // IMU integration
    Eigen::Matrix3f integrateImuRotation();

    // THis published odometry for robot including recent path
    void publishOdometry(const rclcpp::Time &stamp);

    // helper function to publish a path
    void publishPath(Eigen::Matrix4f Transform,
                     std::string frame_id,
                     const rclcpp::Time &stamp);

    // helper function to publish a transform
    void publishTransform(Eigen::Matrix4f Transform,
                          std::string frame_id,
                          std::string child_frame_id,
                          const rclcpp::Time &stamp);

    // helper function to retrieve the static TFs to resolve
    // odom frame -> base_link frame -> lidar frame -> imu frame
    void initStaticTF();

    // Watchdor timer
    void watchdogCheck();

    // distance stats for dynamic TF
    void calculateStats(
        double Value,
        double* MeanValue, double* sigmaValue,
        double Alpha
        );

    // transform
    pcl::PointCloud<pcl::PointXYZI>::Ptr TFPointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const Eigen::Matrix4f &T);

    // Crop cloud using distance method
    pcl::PointCloud<pcl::PointXYZI>::Ptr CropDistance(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        double Distance);

private:    // private class variables

    // ---------------------
    // ROS2 communications
    // ---------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ---------------------
    // PCL
    // ---------------------

    pcl::VoxelGrid<pcl::PointXYZI> vg_filtered_; // voxel filter for scan
    pcl::VoxelGrid<pcl::PointXYZI> vg_local_map_; // voxel filter for local map
    // The current scan translated filtered by voxel_laef
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_;

    // local cloud map (sliding local map that contains the last 'n' scans)
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_;
    // The current scan trasnformed from l2lidar_frame -> base_link frame
    pcl::PointCloud<pcl::PointXYZI>::Ptr base_link_scan_;
    // ICP for matching odom_scan_ against lcoal_map_ (odom frame)
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    // sliding map buffer (base_link frame)
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> scan_queue_;

    // ---------------------
    // Pose tracking
    // ---------------------
    // all transforms start as identity
    // and must be changed with data that is read in
    //Eigen::Matrix4f T_map_base_ = Eigen::Matrix4f::Identity();   // robot pose in map frame
    ///Eigen::Matrix4f T_map_odom_ = Eigen::Matrix4f::Identity();   // odom frame in map frame
    Eigen::Matrix4f T_odom_base_ = Eigen::Matrix4f::Identity();  // odom frame in base frame
    //Eigen::Matrix4f T_odom_lidar_ = Eigen::Matrix4f::Identity(); // odom -> l2liar_frame
    Eigen::Matrix4f T_base_lidar_ = Eigen::Matrix4f::Identity(); // base_link -> l2lidar_frame
    Eigen::Matrix4f T_lidar_imu_ = Eigen::Matrix4f::Identity();  // l2lidar_frame -> l2lidar_imu
    Eigen::Matrix4f T_base_imu_ = Eigen::Matrix4f::Identity();   // base_link -> l2lidar_imu
    Eigen::Matrix3f R_base_imu_ = Eigen::Matrix3f::Identity();   // rotation base_link -> l2lidar_imu
    Eigen::Matrix4f lastAlignedTF_ = Eigen::Matrix4f::Identity(); // used to calculate change in pose/position
                                                                  // since the last ICP align
    Eigen::Matrix4f deltaTransform_ = Eigen::Matrix4f::Identity();// change in pose since last scan use as guess

    std::string odometry_frame_id_; // odometry frame ID
    std::string robot_frame_id_; // robot frame ID
    std::string submap_crop_namespace_; // submap crop box namespace

    // ---------------------
    // Initial map accumulation, only used during the initial map accumulation phase
    // ---------------------
    int init_scans_{25};    // number of scans in initial map (config param)
    int local_map_max_size_; // size of local_map that triggers resize (config param)
    int init_scans_total {0};   // number of scans currently in the initial map accumulation
    bool map_initialized_ {false};  // initial local map accumulation completed

    // ---------------------
    // sliding map trimming instructions
    // ---------------------
    float local_map_radius_ = 5.0f; // meters (config param)
    int max_scan_queue_ = 45;   // max # scans to keep in queue  (config param)
    int scan_trim_size_ = 30;   // when max scan in queue is reached reduce
                                // to scan_trim_size_ by deleting the oldest scans (config param)

    // ---------------------
    // ICP parameters
    // ---------------------
    double voxel_leaf_;  // voxel leaf size used in downsample filtering (config param)
    double correspondence_;// maximum correspondence distance in meters (config param)
    int icp_iterations_; // ICP max iterations parameter (config param)
    double epsilon_; // ICP epsilon parameter (config param)
    double local_submap_distance_; // size in meters of crop box for local_map_ used in ICP (config param)
    double fitness_score_; // worst fitness value that is still acceptable as fit from ICP (config param)
                           // The larger the number the worse then fit (mean squared error of the fit)

    // These are stats that allow dynamic calculation of noise when stationary
    double avgDistance_ {0.0}; // This is the average change is distance for the robot
    double sigmaDistance_ {0.0}; // This is the variance of avgDistance_
    // This is an estimate of the max movement noise from the ICP process
    // Movements under this size will use Identity for the quess rather than deltaTransform
    // This is likely that there is no real robot translation movement.
    // This does not directly effects the robot odometry results other than to reduce drift
    // when the robot is not moving.
    double max_noiseDistance_ {0.03}; // in meters

    //------------------------------------------
    // IMU support (not yet implemented, skeleton only)
    //------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    rclcpp::Time last_scan_time_;
    std::mutex imu_mutex_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;    // IMU message queue
    int max_imu_queue_ {400};

    // ---------------------
    // TF2
    // ---------------------
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool static_tf_ready_{false};
    // retry timer
    rclcpp::TimerBase::SharedPtr tf_timer_;
    int tf_retry_count_{0};
    int tf_max_retries_{50};

    // ---------------------
    // Diagnostic publishers for rViz2
    // ---------------------

    // aligned point cloud publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;

    // path publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    // bounding ICP box (crop box) publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr crop_marker_pub_;

    // watchdog
    rclcpp::Time last_msg_time_;
    long watchdog_timeout_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    bool shutdown_triggered_{false};

    int64_t NumberOfScans_ {0};
};
