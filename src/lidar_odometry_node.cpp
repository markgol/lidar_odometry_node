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
//		Initial target hardware is RPI5 (ARM64) and an x86_64
//
//      ICP processing pipeline
//
//        PointCloud2 (/points
//            │
//        convert to pcl
//            │
//        voxel filter
//            │
//        initial local map accumulation (skipped once completed)
//            │
//        crop local submap (does not use  pcl::CropBox)
//            │
//        ICP align
//            │
//       convergence/fitness validation
//            │
//        pose update
//            │
//        local map update (when map reaches max size, trim using size scan queue)
//            │
//        publish odometry
//            │
//        publish TFs
//            |
//        publish diagnostics
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
    declare_parameter<int>("local_map_max_size", 400000);

    declare_parameter<double>("voxel_leaf", 0.03); // 0.02 to 0.05
    declare_parameter<double>("correspondence", 1.0); // meters
    declare_parameter<double>("epsilon", 1.0e-6);
    declare_parameter<double>("local_submap_distance", 4.0);   // meters
    declare_parameter<bool>("EnableCropDistance", false);
    declare_parameter<double>("fitness_score", 0.3); // LSQ fit mean squared distance in meters
    declare_parameter<double>("max_noiseDistance", 0.03); // distance in meters
    declare_parameter<int>("icp_iterations", 25); // 20-50
    declare_parameter<int>("max_scan_queue", 45); // 20-50
    declare_parameter<int>("max_imu_queue", 500); // 270-1000
    declare_parameter<int>("scan_trim_size", 30); // 20-50
    declare_parameter<std::string>("odometry_frame_id", "odom");
    declare_parameter<std::string>("robot_frame_id", "base_link");
    declare_parameter<std::string>("submap_crop_namespace", "submap_crop");
    declare_parameter<long>("watchdog_timeout", 40);

    // Number of static position scans to acquire for the initial local map
    // before processing starts
    get_parameter("init_scans", init_scans_);

    // max size of local map size
    get_parameter("local_map_max_size", local_map_max_size_);

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
    get_parameter("max_scan_queue", max_scan_queue_);
    get_parameter("max_imu_queue", max_imu_queue_);
    get_parameter("scan_trim_size", scan_trim_size_);
    get_parameter("odometry_frame_id", odometry_frame_id_);
    get_parameter("robot_frame_id", robot_frame_id_);
    get_parameter("submap_crop_namespace", submap_crop_namespace_);

    // --------- Watchdog timer settings---------------
    get_parameter("watchdog_timeout", watchdog_timeout_); // in seconds

    // init scan_queue_ to empty
    scan_queue_.clear();

    // subscribe to the l2lidar_node publisher /points
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points",
        rclcpp::SensorDataQoS(),    // this is important, do not change
        std::bind(&lidar_odometry_node::cloudCallback, this, std::placeholders::_1)
    );

    // subcribe to the l2lidar_node publisher /imu
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS(),    // this is important, do not change
        std::bind(&lidar_odometry_node::imuCallback, this, std::placeholders::_1));

    // create new publisher /odom
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10); // queue size = 10

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // local cloud map (sliding local map that contains the last 'n' scans)
    // overtime this map moves with the robot so its frame is base_link
    local_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // The current scan translated filtered by voxel_laef (base_link frame)
    filtered_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // The current scan transformed from l2lidar_frame -> base_link frame
    base_link_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // clear the scan queue
    scan_queue_.clear();

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

    // diagnostic publisher for aligned frame
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/aligned_scan", 10); // queue size = 10
    //last_publish_time_ = now();

    // diagnostic path publisher
    //  In development this will be used for publishing both map and odom paths
    //  In production only map path will be used
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/path", 10); // queue size = 10

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
    //  IMPORTANT:  The point cloud scan is pose (rotation) corrected before being
    //  received.  This is optional in the L2lidar_node.
    //  If it is applied then only translation position correction is required
    //  If it s not applied that both rotation annd translation will need to be estimated
    //*************************************************************************

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *scan);
    // scan is in the l2lidar_frame, it will need to be translated into the local
    // map reference frame odom
    // Note: odom is initially set to t_base_lidar_
    //
    // The odom frame is only different from the T_base_lidar frame when there is
    // movement in the robot.
    //
    // Since the odom frame utilizes a sliding local map if the robot stops moving
    // odom frame will become the same as the base_link frame after 'n' scans
    // With movement the odom frame will lag the base_link frame.
    //
    // A map frame relative to the staring position of the robot is needed to
    // accurately portray the actual robot pose and path.  The current path in the
    // map frame needs to be adjusted by the delta of the new scan to the position
    // returned by the last scan (deltaTransform)
    //

    // downsample scan to filtered_scan_
    //filtered_scan_ = scan; // This line is only used for diagnostics
                             // and the filtered lines of code commented
    vg_filtered_.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    vg_filtered_.setInputCloud(scan);
    vg_filtered_.filter(*filtered_scan_); // this is still in l2lidar frame


    // ---------------------------------------------
    // Transform filtered_scan_ from l2lidar_frame -> base_link frame
    //  result is base_link_scan_
    // ---------------------------------------------
    base_link_scan_ = TFPointCloud(filtered_scan_, T_base_lidar_);
    // pcl::transformPointCloud(*filtered_scan_,
    //                          *base_link_scan_,
    //                          T_base_lidar_); // at start T_odom_lidar  = T_base_lidar

    // Accumulate frames for the initial local map
    // For the Unitree L2 this should be 20-50 aggregated scans
    // while in static position
    if (!map_initialized_)
    {
        *local_map_ += *base_link_scan_;

        init_scans_total++;

        if (init_scans_total >= init_scans_)
        {
            map_initialized_ = true;
            // The initial local map is complete
            // There are currently there are 5 frame references:
            //      odom (local space with origin at robot starting point)
            //      base_link (this is the robot origin)
            //      lidar
            //      imu
            //
            //  At the start the local map is complete these 3 are the
            //  same and initially set to identity (since there is no real world reference)
            //      odom -> base   = I
            //      it changes over time by change in position by robot
            //
            //  The static_tf_ready_ processing ensures:
            //          base->lidar->imu are defined.
            //  They are fixed and and never change.
            //
            //  When SLAM with preexisting world map is introduced
            //  There will be a map -> odom transform in addition
            //  to these.

            // This is just diagnostic
            RCLCPP_INFO(get_logger(), "Initial map created with %ld points",
                        local_map_->size());
        }
        // We have a complete local_map_ start processing on the next scan.
        return;
    }

    // only get here after initial local map creation

    // crop local map by distance for robot
    auto start = std::chrono::steady_clock::now();
    double msec;

    pcl::PointCloud<pcl::PointXYZI>::Ptr submap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr subscan;

    if(EnableCropDistance_) {
        submap = CropDistance(local_map_, local_submap_distance_);
        subscan = CropDistance(base_link_scan_, local_submap_distance_);
        auto end = std::chrono::steady_clock::now();
        msec = std::chrono::duration<double, std::milli>(end-start).count();

        RCLCPP_INFO(get_logger(), "Crop time: %.1f ms", msec);
    }

    // Make intial guess for scan position
    // Currently this is using the ICP matching process to estimate movement.
    // Use of motor encoders may be more reliable predictor for movement including
    // determinination if the robot is stationary.

    // This initially is a guess about the translation movement
    // in x,z,y since the last frame.
    // Assumption:  incoming is pose rotation corrected already using
    // imu data in the l2lidar_node.  If this is true DO NOT double apply
    // rotation.
    // in the first pass thru deltaTransform  = I
    // guess = last position plus deltaTransform;
    // This assumes same direction and velocity

    // calculate distance to get running stats
    // on the translation component of deltaTransform
    Eigen::Vector3f dt = deltaTransform_.block<3,1>(0,3);
    double distance = dt.norm(); // in meters
    // time how long ICP takes to process

    calculateStats(
        distance,
        &avgDistance_, &sigmaDistance_,
        1.0/(5.55*3.0)  // last 3 seconds running stats
    );

    RCLCPP_INFO(get_logger(),
                "odom distance noise: mean %.5f  std dev: %.5f",
                avgDistance_,
                sqrt(sigmaDistance_));

    // use 2 std dev above mean translation delta as limit
    double noise_limit;
    noise_limit = avgDistance_+(2.0*sqrt(sigmaDistance_));

    // make a guess
    noise_limit = std::min(noise_limit,max_noiseDistance_);

    Eigen::Matrix4f quess;
    if(distance > noise_limit) {
        // robot likely changed position
        quess = deltaTransform_;
    } else {
        // no siginficant change in robot position
        quess = Eigen::Matrix4f::Identity();
    }
    // End of guess section

    RCLCPP_INFO(get_logger(),
                "submap map size: %ld  Scan size: %ld",
                submap->size(),
                base_link_scan_->size());

    // --- ICP ALIGNMENT ---
    // current scan in base_link frame reference
    if(EnableCropDistance_) {
        icp.setInputSource(subscan);
        icp.setInputTarget(submap); // align to the local_map_ (submap)
    } else {
        icp.setInputSource(base_link_scan_);
        icp.setInputTarget(local_map_); // align to the local_map_ (submap)
    }

    pcl::PointCloud<pcl::PointXYZI> aligned;

    // time how long ICP takes to process
    start = std::chrono::steady_clock::now();

    // align using current odom pose guess
    icp.align(aligned,quess); // start from internal starting guess
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
    // not solving it precisely which ould be extremely slow.
    // This spareness results in poorer fitness scores at times.
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

    // *** This was just for convenient diagnostics breakpoint ***
    // if(NumberOfScans_>20) {
    //     if((NumberOfScans_%100)==0) {
    //         RCLCPP_INFO(get_logger(),
    //                 "Number of scans: %ld",
    //                 NumberOfScans_);
    //     }
    // }
    // *** end of diagnostic section ***

    // --- UPDATE POSES ---
    // The aligned scan is in the base_link frame reference and is already translated
    // to the best fit within the base_link local_map_ frame.
    // The T_alignment value is always the absolute transform used
    Eigen::Matrix4f T_alignment = icp.getFinalTransformation(); // transform from ICP

    // Subtract current alignment from previous alignment
    // This gives the delta movement from the last scan
    // This could be as part of guess for next ICP align
    // assuming a constant velocity and direction.
    // Note: T_alignment and lastAlignedTF_ are TFs referencing alignment transforms within
    // odom frame. There are abolsute TFs between the aligned frame the odom frame.
    // They are only referenced to the odom frame.
    // deltaTransform is a change in position and is not specific to a given frame reference.
    // It can be used to update a guess and update the map to robot position
    deltaTransform_ = T_alignment * lastAlignedTF_.inverse(); // change from last frame
    lastAlignedTF_ = T_alignment; // update last frame to current frame
    //
    // Useful transforms for deltaTransform
    //
    // This is the chagne in distance from last postion in meters
    //  Eigen::Vector3f dt = deltaTransform_.block<3,1>(0,3);
    //  double translation = dt.norm();
    //
    // This is the rotation magnitude in radians
    //  Eigen::Matrix3f R = deltaTransform_.block<3,3>(0,0);
    //  Eigen::Quaternionf q(R);
    //  double angle = 2.0 * std::acos(std::abs(q.w()));
    //
    // for example:
    // if(distance<TBD) {
    //      robot is not likely to have moved.
    //      Must decide whether or not to update translation
    //

    // Odometry
    // T_base_lidar_ is the relationship between the
    // (local_map_) frame and the lidar frame
    // since the local_map_ is a sliding map that moves over time
    // If the robot becomes stationary then its position
    // will match the base (robot) position
    // T_alignment is the aligned scan position in the base_link frame
    // So it becomes our new scanner position
    T_base_lidar_ = T_base_lidar_ * T_alignment; // the new robot position

    // current position in the map is integration of the robot incremental
    // movement from scan to scan
    // update robot position change in position to update location in map frame
    T_odom_base_ = T_odom_base_ * deltaTransform_;

    // --- UPDATE SCAN QUEUE (sliding map) ---
    // push the aligned scan into the queue
    // The scan queue stays time ordered
    // push this aligned scan into the queue
    // 'aligned' scan is in the odom frame
    scan_queue_.push_back(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(aligned));

    // keep queue size constant
    // The front of the queue is the oldest scan frame
    // so remove it
    while(scan_queue_.size()> max_scan_queue_) {
        scan_queue_.pop_front();
    }

    // --- UPDATE LOCAL MAP ---
    // Also add the aligned scan to the local_map_
    *local_map_ += aligned;

    // trim local map to max local map size
    // trimming is bascially being controled by the local_map_max_size_
    // The local_map_ grows by the aligned.size() each scan.
    // As the loca_map_ size grows the ICP align time also grows
    // The scan_trim_size_ (#scans) must be less than local_map_max_size_/max_scan_size
    // The closer you are to this will causethe function to update the local_map_ more oftten
    // from the saved scan_queue.  For example if the local_map_max_size_/max_scan_size
    // is about 50 scans and scan_trim_size is 35 then the local_map_ will get replaced
    // by the scan_queue about every 15-20 scans.  The primary purpose of this is to keep
    // the ICP time fairly constnat.  Too few scan will cause more iterations to solve
    // becuase of the sparseness of the local_map_.

    if (local_map_->size() > local_map_max_size_)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);

        // copy only newest scans from scan_queue_ to tmp
        // tmp is rebuild of the local_map_ using the last scan_trim_size_ scans in the queue
        // scan_queue[0] is the oldest scan and scan_queue[scan_queue.size()-1] is th eoldest scan.
        int scans_to_copy = std::min((int)scan_queue_.size(), scan_trim_size_);
        for (int i = scan_queue_.size() - scans_to_copy; i < (int)scan_queue_.size(); ++i)
        {
            *tmp += *scan_queue_[i];
        }

        local_map_ = tmp; // replace map with new map of the latest scans

        // downsample local_map_ if still too large
        if (local_map_->size() > local_map_max_size_)
        {
            vg_local_map_.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
            vg_local_map_.setInputCloud(local_map_);
            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZI>);
            vg_local_map_.filter(*downsampled_map);
            local_map_ = downsampled_map;
        }

        RCLCPP_INFO(get_logger(),
                    "Local map trimmed: current size = %ld points, scans = %ld",
                    local_map_->size(),
                    (long)scans_to_copy);
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
