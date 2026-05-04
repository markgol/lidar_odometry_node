//--------------------------------------------------------
//
//  lidar_odometry_node
//  Author: Mark Stegall
//  Module: lidar_odometry_node.cpp
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
//      ROS2 node sources:
//          include/lidar_odometry.h
//          src/lidar_odometry_node.cpp
//          src/lidar_base_imu.cpp
//          src/odom_TFs_publishers.cpp
//
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//		Initial target hardware is RPI5 (ARM64) and x86_64
//
//
//      TRANSFORM TREE
//
//      map(real world not yet implemented)
//        └──  odom (local map with origin at the starting point of the robot)
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
//          frame id: base
//
//      topic /path
//          frame_id: odom
//
//      topic /path
//          frame_id: base_link
//
//      topic /odom
//          frame_id: odom
//          child_frame_id: l2lidar_frame
//
//      topic /submap_crop_marker
//          frame_id: base_link
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
//      The initial pose for the local self determined odom frame is the same as base_linK.
//      The initial local map is composed on 'n' scans in this pose.
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
//      V0.5.0  2026-04-06  Added keyframe publshing for SLAM
//                          Updated QOS settings for publishers and subscribers
//                          Added detection of "/imu/data" topic (from l2lidar_node)
//                          Currently this node does not process the L2 IMU data but the
//                          skeletal framwork is in place.
//      V0.5.1  2026-04-25  Added config parameter for odom topic
//      V0.6.0  2026-04-28  Changed local map to be initial established anchor map then
//                          create queue of immutable submaps created from new position keyframes
//                          use immutable map from the 'n' closest immutable maps for ICP matching.
//                          Added isStationary state.  This is to reduce drift due to noise when
//                          the robot is not moving.
//
//      QtCreator IDE was used in the development
//      This package has NO Qt depdendencies or libraries
//      It uses CMakeList.txt for configuration
//
#include "lidar_odometry_node.h"

//--------------------------------------------------------
//  main()
//  This is that standard ROS2 app main for publisher node
//--------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<lidar_odometry_node>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

//--------------------------------------------------------
//  watchdogCheck
//  watchdog timer timeout ends app
//--------------------------------------------------------
void lidar_odometry_node::watchdogCheck()
{
    if (shutdown_triggered_) return;

    auto elapsed = this->get_clock()->now() - last_msg_time_;

    // The wathdog timer triggers no subscriber topic data is received
    // for a period of time (watchdog_timeout_).
    // This can occur when the lidar sensor is powered down or has some type
    // of fault.  It also occurs if the l2lidar_node exits or hangs.
    if (elapsed > rclcpp::Duration::from_seconds(watchdog_timeout_)) {
        shutdown_triggered_ = true;

        RCLCPP_ERROR(get_logger(),
                     "Watchdog timeout (%.1f sec) → shutting down",
                     elapsed.seconds());

        rclcpp::shutdown();
    }
}

//--------------------------------------------------------
//  lidar_odometry_node constructor
//--------------------------------------------------------
lidar_odometry_node::lidar_odometry_node()
        : Node("lidar_odometry_node")
{
    // parameters from yaml configuration file
    declare_parameter<int>("init_scans", 20);

    declare_parameter<double>("voxel_leaf", 0.03); // 0.02 to 0.05
    declare_parameter<double>("correspondence", 1.0); // meters
    declare_parameter<double>("epsilon", 1.0e-6);
    declare_parameter<double>("local_submap_distance", 4.0);   // meters
    declare_parameter<bool>("EnableCropDistance", true);
    declare_parameter<double>("fitness_score", 0.3); // LSQ fit mean squared distance in meters
    declare_parameter<double>("max_noiseDistance", 0.005); // distance in meters
    declare_parameter<int>("icp_iterations", 25); // 20-50
    declare_parameter<int>("max_scan_queue", 45); // 20-50
    declare_parameter<int>("scan_trim_size", 30); // 20-50
    declare_parameter<std::string>("odometry_frame_id", "odom");
    declare_parameter<std::string>("odom_topic", "odom");
    declare_parameter<std::string>("robot_frame_id", "base_link");
    declare_parameter<long>("watchdog_timeout", 60);
    declare_parameter<double>("keyframe_translation_thresh", 0.5); //0.4-0.7m
    declare_parameter<double>("keyframe_rotation_thresh", 0.3); // radians (~17 degrees)
    declare_parameter<double>("keyframe_min_translation_for_rotation", 0.1); // 0.1m
    declare_parameter<double>("keyframe_last_frame_time", 2.0); // 2 sec
    declare_parameter<std::string>("keyframe_pose_topic", "/lidar/keyframes/pose");
    declare_parameter<std::string>("keyframe_point_topic", "/lidar/keyframes/cloud");
    declare_parameter<std::string>("aligned_scan_topic", "/aligned_scan");
    declare_parameter<std::string>("path_topic", "/path");
    declare_parameter<int>("max_imu_queue", 500); // 270-1000
    // V0.6.0 new configuration parameters
    declare_parameter<int>("max_submaps", 8); // 5-20
    declare_parameter<double>("submap_radius", 4.0); // distance in meters, 4.0 inside, 10.0 outside
    declare_parameter<int>("max_selected_submaps", 3); // 3-5
    declare_parameter<double>("submap_dist_thresh", 0.5); // distance in meters
    declare_parameter<double>("submap_yaw_thresh", 10.0 * M_PI / 180.0); // yaw rotation threshold (10 degrees)
    declare_parameter<double>("max_predict_dist", 1.5); // distance in meters
    declare_parameter<double>("max_rotation_step", 45.0 * M_PI / 180.0); // max yaw rotation (45 degrees)
    declare_parameter<int>("submap_max_points", 400000); // max yaw rotation (45 degrees)

    // Number of static position scans to acquire for the initial local map
    // before processing starts
    get_parameter("init_scans", init_scans_);

    // V0.6.0 new configuration parameters
    get_parameter("max_submaps", max_submaps_);
    get_parameter("submap_radius", submap_radius_);
    get_parameter("max_selected_submaps", max_selected_submaps_);
    get_parameter("submap_dist_thresh", submap_dist_thresh_);
    get_parameter("submap_yaw_thresh", submap_yaw_thresh_);
    get_parameter("max_predict_dist", max_predict_dist_);
    get_parameter("max_rotation_step", max_rotation_step_);
    get_parameter("submap_max_points", submap_max_points_);

    //voxel downsample size
    get_parameter("voxel_leaf", voxel_leaf_);

    // ICP parameters
    get_parameter("correspondence", correspondence_);
    get_parameter("epsilon", epsilon_);
    get_parameter("local_submap_distance", local_submap_distance_);
    get_parameter("EnableCropDistance", EnableCropDistance_);
    get_parameter("fitness_score", fitness_score_);
    get_parameter("max_noiseDistance", max_noiseDistance_);    
    get_parameter("icp_iterations", icp_iterations_);
    get_parameter("max_imu_queue", max_imu_queue_);
    get_parameter("odometry_frame_id", odometry_frame_id_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("robot_frame_id", robot_frame_id_);

    // Keyframe parameters
    get_parameter("keyframe_translation_thresh", keyframe_translation_thresh_);
    get_parameter("keyframe_rotation_thresh", keyframe_rotation_thresh_);
    get_parameter("keyframe_min_translation_for_rotation", keyframe_min_translation_for_rotation_ );
    get_parameter("keyframe_last_frame_time", keyframe_last_frame_time_);
    get_parameter("keyframe_pose_topic", keyframe_pose_topic_);
    get_parameter("keyframe_point_topic", keyframe_point_topic_);

    // --------- Watchdog timer settings --------------
    get_parameter("watchdog_timeout", watchdog_timeout_); // in seconds

    // --------- debug publishers ---------------------
    get_parameter("aligned_scan_topic", aligned_scan_topic_);
    get_parameter("path_topic", path_topic_);

    // subscribe to the l2lidar_node publisher /points
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points",
        rclcpp::SensorDataQoS(),    // this is important, do not change
        std::bind(&lidar_odometry_node::cloudCallback, this, std::placeholders::_1)
    );

    // subcribe to the l2lidar_node publisher /imu
    // if the l2lidar_node is publishing IMU data
    {
        auto publishers = this->get_publishers_info_by_topic("/imu/data");

        if (publishers.empty()) {
            // Publisher does not exist
            RCLCPP_INFO(get_logger(), "Lidar Odometry Node no IMU subscription");
        } else {
            // Publisher exists
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                "/imu",
                rclcpp::SensorDataQoS(),    // this is important, do not change
                std::bind(&lidar_odometry_node::imuCallback, this, std::placeholders::_1));
        }
    }

    // create new publisher odom_topic_
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10); // queue size = 10

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // local cloud map (sliding local map that contains the last 'n' scans)
    // overtime this map moves with the robot so its frame is base_link
    local_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // The current scan translated filtered by voxel_laef (base_link frame)
    filtered_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // The current scan transformed from l2lidar_frame -> base_link frame
    base_link_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // clear the scan queue
    // ??? scan_queue_.clear();

    // ICP parameters
    icp.setTransformationEpsilon(epsilon_);
    icp.setEuclideanFitnessEpsilon(epsilon_);
    icp.setMaximumIterations(icp_iterations_);
    icp.setMaxCorrespondenceDistance(correspondence_);

    // buffers for static TF2 handler
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Start timer to acquire static TF
    initStaticTF(); // start timer, retry up to 50 times

    last_msg_time_ = this->get_clock()->now();

    // start wathdog timer
    watchdog_timer_ = this->create_wall_timer(std::chrono::seconds(1),  // coarse check is sufficient
                                              [this]() { watchdogCheck(); }
                                              );

    // Keyframe publishers
    keyframe_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        keyframe_pose_topic_, 10);

    keyframe_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        keyframe_point_topic_, 10);

    last_keyframe_time_ = this->get_clock()->now();

    // diagnostic publisher for aligned frame
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        aligned_scan_topic_, 10); // queue size = 10
    //last_publish_time_ = now();

    // diagnostic path publisher
    //  In development this will be used for publishing both map and odom paths
    //  In production only map path will be used
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        path_topic_, 10); // queue size = 10

    RCLCPP_INFO(get_logger(), "Lidar Odometry Node Initialized");
}

//--------------------------------------------------------
//  cloudCallback
//--------------------------------------------------------
void lidar_odometry_node::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Reset watchdog timer
    last_msg_time_ = this->get_clock()->now();

    // safety check, Make sure the TFs are read before processing
    if(!static_tf_ready_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Waiting for static TF before processing scans");
        return;
    }

    //*************************************************************************
    //  Data comes from the L2lidar_node puplisher
    //  IMPORTANT:  The point cloud scan should NOT BE POSE(ROTATION) CORRECTED
    //  before being received.  This is optional in the L2lidar_node.  It should
    //  be set to false.
    //*************************************************************************

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *scan);
    // scan is in the l2lidar_frame, it will need to be translated into the correct
    // frame reference
    // Note: odom is initially set to t_base_lidar_
    //

    // ---------------------------------------------
    // Transform scan from l2lidar_frame -> base_link frame
    //  result is base_link_scan_
    // ---------------------------------------------
    base_link_scan_ = TFPointCloud(scan, T_base_lidar_);

    // Accumulate frames for the initial local map
    // For the Unitree L2 this should be 20-50 aggregated scans
    // while in static position
    if (!map_initialized_)
    {
        *local_map_ += *base_link_scan_; // full scan, not downsampled

        init_scans_total++;

        if (init_scans_total >= init_scans_)
        {
            map_initialized_ = true;
            // The initial anchor map is complete
            // There are currently there are 4 frame references:
            //      odom (local space with origin at robot starting point)
            //      base_link (this is the robot origin)
            //      lidar
            //      imu
            //
            //  At the start the anchor map is complete the following are the
            //  same and initially set to identity (since there is no real world reference)
            //      odom -> base   = I
            //
            //  The static_tf_ready_ processing ensures:
            //          base->lidar->imu are defined.
            //          These are fixed and and never change.
            //
            //  When SLAM with preexisting world map is introduced
            //  There will be a map -> odom transform in addition
            //  to these.  This is not the responibility of this node.

            // Freeze anchor

            // anchor_submap is immutable and is in odom reference frame
            anchor_submap_.reset(new pcl::PointCloud<pcl::PointXYZI>(*local_map_));
            anchor_ready_ = true;

            // Start active submap
            active_submap_.reset(new pcl::PointCloud<pcl::PointXYZI>);

            last_submap_pose_ = T_odom_base_;

            // This is just diagnostic
            RCLCPP_INFO(get_logger(), "Anchor map created with %ld points",
                        anchor_submap_->size());
        }
        // We have a complete anchor_submap_ start processing on the next scan.
        return;
    }

    // only get here after initial local map creation

    // V0.6.0 changes

    // Make intial guess for scan position
    // Currently this is using the ICP matching process to estimate movement.
    // Use of motor encoders may be more reliable predictor for movement including
    // determinination if the robot is stationary.

    // This initially is a guess about the translation movement
    // since the last frame.

    // In the first pass thru deltaTransform  = Identity
    // guess = last position plus deltaTransform;
    // This assumes same direction and velocity

    // calculate distance to get running stats
    // on the translation component of deltaTransform
    Eigen::Matrix4f deltaMeasured_;  // raw from ICP (never suppressed)

    Eigen::Vector3f dt = deltaMeasured_.block<3,1>(0,3);
    double distance = dt.norm(); // in meters
    // time how long ICP takes to process

    calculateStats(
        distance,
        &avgDistance_, &sigmaDistance_,
        1.0/(5.55*3.0)  // last 3 seconds running stats
        );

    // This will need inclusion of platform status in addition
    // to this calculation
    bool isStationary;

    RCLCPP_INFO(get_logger(),
                "delta distance noise: mean %.5f  std dev: %.5f",
                avgDistance_,
                sqrt(sigmaDistance_));

    Eigen::Matrix4f guess;

    isStationary = (avgDistance_ < max_noiseDistance_) &&
                   (sigmaDistance_ < max_noiseDistance_);

    if(isStationary) {
        guess = lastAlignedTF_;
    } else {
        Eigen::Matrix4f delta_pred = deltaTransform_;
        ClampDelta(delta_pred);
        Eigen::Matrix4f guess = lastAlignedTF_ * delta_pred;
    }
    // End of guess section

    auto target = buildICPTarget(); // made up from submaps or anchor submap at minimum
    if (target->empty()) {
        *target += *anchor_submap_;
    }

    RCLCPP_INFO(get_logger(),
                "target map size: %ld  Scan size: %ld",
                target->size(),
                base_link_scan_->size());

    // --- ICP ALIGNMENT ---

    icp.setInputSource(base_link_scan_);
    icp.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZI> aligned;

    // time how long ICP takes to process
    auto start = std::chrono::steady_clock::now();
    double msec;

    // align using current odom pose guess
    icp.align(aligned,guess); // start from internal starting guess
    // guess should be added when better understood

    auto end = std::chrono::steady_clock::now();
    msec = std::chrono::duration<double, std::milli>(end-start).count();

    RCLCPP_INFO(get_logger(), "ICP time: %.1f ms", msec);

    NumberOfScans_++;

    // check for solution convergence (max iterations)
    if (!icp.hasConverged())
    {
        RCLCPP_WARN(get_logger(), "ICP did not converge");
        return;
    }

    // Check fitness score
    // Note:  the local map is a sparse map in which vertical slices are missing
    // A complete map with no vertical slices missing would take 480 scans.
    // One of the purposes of the ICP align is make a best guess at the solution
    // not solving it precisely which would be extremely slow.
    // This spareness results in poorer fitness scores at times.
    // but not significantly poorer
    double current_fitness_score =icp.getFitnessScore();

    if (current_fitness_score > fitness_score_)
    {
        RCLCPP_WARN(get_logger(), "Bad ICP fitness");
        return;
    }

    RCLCPP_INFO(get_logger(),
                "ICP iterations: %d  fitness: %.6f",
                icp.nr_iterations_,
                current_fitness_score);

    // --- UPDATE POSES ---
    Eigen::Matrix4f T_alignment = icp.getFinalTransformation(); // transform from ICP

    deltaMeasured_ = lastAlignedTF_.inverse() * T_alignment;

    if(isStationary) {
        deltaTransform_ = Eigen::Matrix4f::Identity();

        // Freeze pose
        T_alignment = lastAlignedTF_;

        RCLCPP_WARN(get_logger(), "Stationary: ICP motion suppressed");
    }  else {
        // Subtract current alignment from previous alignment
        // This gives the delta movement from the last scan
        // This is used for of guess to be used in next ICP align
        // assuming a constant velocity and direction.

        // Note: T_alignment and lastAlignedTF_ are TFs referencing alignment transforms within
        // reference frame of the target map.
        // deltaTransform is a change in position and is not specific to a given frame reference.
        // It can be used to update a guess and update the map to robot position
        Eigen::Matrix4f delta = lastAlignedTF_.inverse() * T_alignment;

        // Clamp delta
        ClampDelta(delta);

        // recompute corrected alignment
        T_alignment = lastAlignedTF_ * delta;

        // update state
        deltaTransform_ = delta;
        lastAlignedTF_  = T_alignment;
    }

    T_odom_base_ = T_alignment;

    // accumulate into active submap (this is already odom frame)
    if (!isStationary) {
        *active_submap_ += aligned;
    }

    if (shouldCloseSubmap())
    {
        Submap sm;
        sm.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>(*active_submap_));

        // compute center
        Eigen::Vector3f centroid(0,0,0);
        for (auto& p : sm.cloud->points) {
            centroid += Eigen::Vector3f(p.x, p.y, p.z);
        }
        centroid /= sm.cloud->size();
        sm.center = centroid;

        submaps_.push_back(sm);

        if ((int)submaps_.size() > max_submaps_) {
            submaps_.pop_front();
        }

        active_submap_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        last_submap_pose_ = T_odom_base_;

        RCLCPP_INFO(get_logger(), "Submap closed. Total: %ld",
                    submaps_.size());
    }

    // --- KEYFRAME EXTRACTION ---
    if (shouldCreateKeyframe(T_odom_base_)) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_aligned(
            new pcl::PointCloud<pcl::PointXYZI>);

        // aways use full scan for keyframe, never cropped
        // scan is the full scan but needs to be TF to T_base_lidar
        // base_link_scan_ is downsampled
        pcl::PointCloud<pcl::PointXYZI>::Ptr TFscan(
            new pcl::PointCloud<pcl::PointXYZI>);
        TFscan = TFPointCloud(scan, T_base_lidar_);

        pcl::transformPointCloud(
            *TFscan,
            *full_aligned,
            T_alignment); // now odom_base

        publishKeyframe(
            T_odom_base_,
            full_aligned,
            msg->header.stamp
            );

        last_keyframe_pose_ = T_odom_base_;
        first_keyframe_ = false;

        RCLCPP_INFO(get_logger(), "Keyframe created");
    }

    //------------------------------------------
    // publish TF odom->base_link
    //------------------------------------------
    publishTransform(T_odom_base_,odometry_frame_id_, robot_frame_id_, msg->header.stamp);

    //------------------------------------------
    // publish diagnostic odom->robot path
    //------------------------------------------
    // currrent path behaviour should look like random noise when stationry
    // changes in rotation and translation will show with a biased non
    // random direction which eventually collaped back to noise about the base_link frame
    // when the robot is stationary
    publishPath(T_odom_base_,odometry_frame_id_, msg->header.stamp);

    //------------------------------------------
    // publish odometry
    //------------------------------------------
    publishOdometry(msg->header.stamp);

    // ------------------------------------------------
    // diagnostic Publish aligned cloud for RViz2 visualization
    // ------------------------------------------------
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned, aligned_msg);
    aligned_msg.header.stamp = msg->header.stamp;
    aligned_msg.header.frame_id = robot_frame_id_;
    aligned_cloud_pub_->publish(aligned_msg);
}

//---------------------------------------------------
// ClampDelta
//---------------------------------------------------
void lidar_odometry_node::ClampDelta(Eigen::Matrix4f& delta)
{
    // translation clamp
    Eigen::Vector3f t = delta.block<3,1>(0,3);
    float dist = t.norm();

    if (dist > max_predict_dist_) {
        delta.block<3,1>(0,3) *= (max_predict_dist_ / dist);
        RCLCPP_INFO(get_logger(), "Distance clamped");
    }

    // rotation clamp
    Eigen::Matrix3f R = delta.block<3,3>(0,0);
    Eigen::AngleAxisf aa(R);
    float angle = std::abs(aa.angle());

    if (angle > max_rotation_step_) {
        float scale = max_rotation_step_ / angle;
        aa.angle() *= scale;
        delta.block<3,3>(0,0) = aa.toRotationMatrix();
        RCLCPP_INFO(get_logger(), "Rotation clamped");
    }
}

//--------------------------------------------------------
//  buildICPTarget
//--------------------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_odometry_node::buildICPTarget()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(
        new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Vector3f current_pos = T_odom_base_.block<3,1>(0,3);

    // --- Select nearest submaps ---
    std::vector<std::pair<float,int>> dist_idx;

    for (int i = 0; i < (int)submaps_.size(); ++i) {
        float d = (submaps_[i].center - current_pos).norm();
        dist_idx.emplace_back(d, i);
    }

    std::sort(dist_idx.begin(), dist_idx.end());

    int used = 0;
    for (auto& di : dist_idx) {
        if (di.first > submap_radius_) break;

        *target += *(submaps_[di.second].cloud);
        used++;
        if (used >= max_selected_submaps_) break;
    }

    // --- Optionally include anchor ---
    if (anchor_ready_) {
        float d_anchor = current_pos.norm(); // assuming start at origin
        if (d_anchor < 15.0f) {
            *target += *anchor_submap_;
        }
    }

    return target;
}

//--------------------------------------------------------
//  shouldCloseSubmap
//--------------------------------------------------------
bool lidar_odometry_node::shouldCloseSubmap()
{
    if(active_submap_->size() > submap_max_points_) {
        return true;
    }

    Eigen::Vector3f t_curr = T_odom_base_.block<3,1>(0,3);
    Eigen::Vector3f t_last = last_submap_pose_.block<3,1>(0,3);

    float dist = (t_curr - t_last).norm();

    Eigen::Matrix3f R_curr = T_odom_base_.block<3,3>(0,0);
    Eigen::Matrix3f R_last = last_submap_pose_.block<3,3>(0,0);

    Eigen::Matrix3f R_delta = R_last.transpose() * R_curr;
    float yaw = atan2(R_delta(1,0), R_delta(0,0));

    return (dist > submap_dist_thresh_) ||
           (std::abs(yaw) > submap_yaw_thresh_);
}
