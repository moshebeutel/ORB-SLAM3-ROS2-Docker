/**
 * @file rgbd-slam-node.cpp
 * @brief Implementation of the BaseSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "base-slam-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    BaseSlamNode::BaseSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_ROS2")
    {
        // ROS Publishers
        
        

        //---- the following is published when a service is called
        mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        visibleLandmarksPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visible_landmarks", 10);
        visibleLandmarksPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("visible_landmarks_pose", 10);
        slamInfoPub_ = this->create_publisher<slam_msgs::msg::SlamInfo>("slam_info", 10);
        //---- the following is published continously
        mapDataPub_ = this->create_publisher<slam_msgs::msg::MapData>("map_data", 10);
        robotPoseMapFrame_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose_slam", 10);

        // ROS Subscribers
        // Declare parameters (topic names)
        this->declare_parameter("rgb_image_topic_name", rclcpp::ParameterValue("camera/image_raw"));
        this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu"));
        this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));

        rclcpp::QoS qos(10);
        // qos.best_effort(); 
        qos.keep_last(1000).reliable();// or best_effort()
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(this->get_parameter("imu_topic_name").as_string(), qos, std::bind(&BaseSlamNode::ImuCallback, this, std::placeholders::_1));
        qos.keep_last(100).reliable();
        rgbSub_ = this->create_subscription<sensor_msgs::msg::Image>(this->get_parameter("rgb_image_topic_name").as_string(), qos, std::bind(&BaseSlamNode::RGBCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic_name").as_string(), qos, std::bind(&BaseSlamNode::OdomCallback, this, std::placeholders::_1));
        
        // Services
        getMapDataService_ = this->create_service<slam_msgs::srv::GetMap>("orb_slam3/get_map_data", std::bind(&BaseSlamNode::getMapServer, this,
                                                                                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        pointsInViewCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        getMapPointsService_ = this->create_service<slam_msgs::srv::GetLandmarksInView>("orb_slam3/get_landmarks_in_view", std::bind(&BaseSlamNode::getMapPointsInViewServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, pointsInViewCallbackGroup_);
        mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapPointsService_ = this->create_service<slam_msgs::srv::GetAllLandmarksInMap>("orb_slam3/get_all_landmarks_in_map", std::bind(&BaseSlamNode::publishMapPointCloud, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, mapPointsCallbackGroup_);
        resetLocalMapSrv_ = this->create_service<std_srvs::srv::SetBool>("orb_slam3/reset_mapping", std::bind(&BaseSlamNode::resetActiveMapSrv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, mapPointsCallbackGroup_);

        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("robot_base_frame", "base_footprint");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("robot_z", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_z", robot_z_);

        // Declare and get the quaternion components
        this->declare_parameter("robot_qx", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qx", robot_qx_);

        this->declare_parameter("robot_qy", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qy", robot_qy_);

        this->declare_parameter("robot_qz", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qz", robot_qz_);

        this->declare_parameter("robot_qw", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_qw", robot_qw_);

        // Create and populate the Pose message
        geometry_msgs::msg::Pose initial_pose;
        initial_pose.position.x = robot_x_;
        initial_pose.position.y = robot_y_;
        initial_pose.position.z = robot_z_;
        initial_pose.orientation.x = robot_qx_;
        initial_pose.orientation.y = robot_qy_;
        initial_pose.orientation.z = robot_qz_;
        initial_pose.orientation.w = robot_qw_;

        this->declare_parameter("odometry_mode", rclcpp::ParameterValue(true));
        this->get_parameter("odometry_mode", odometry_mode_);

        this->declare_parameter("publish_tf", rclcpp::ParameterValue(true));
        this->get_parameter("publish_tf", publish_tf_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("do_loop_closing", rclcpp::ParameterValue(true));
        this->get_parameter("do_loop_closing", do_loop_closing_);

        // Timers
        mapDataCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapDataTimer_ = this->create_wall_timer(std::chrono::milliseconds(map_data_publish_frequency_), std::bind(&BaseSlamNode::publishMapData, this), mapDataCallbackGroup_);

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, do_loop_closing_, initial_pose, global_frame_, odom_frame_id_, robot_base_frame_id_);

        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    BaseSlamNode::~BaseSlamNode()
    {
        rgbSub_.reset();
        interface_.reset();
        imuSub_.reset();
        odomSub_.reset();
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void BaseSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        // RCLCPP_INFO(this->get_logger(), "ImuCallback");
        // RCLCPP_INFO(this->get_logger(), "IMU received: t=%.6f",
        // msgIMU->header.stamp.sec + msgIMU->header.stamp.nanosec * 1e-9);
        // push value to imu buffer.
        interface_->handleIMU(msgIMU);
    }

    void BaseSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if (odometry_mode_)
        {   // populate map to odom tf if odometry is being used
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }

    // std::vector<ORB_SLAM3::IMU::Point> BaseSlamNode::extractImuMeasurements(double tIm)
    // {
    //     std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    //     std::lock_guard<std::mutex> lock(interface_->bufMutex_);

    //     while (!interface_->imuBuf_.empty())
    //     {
    //         auto imuMsg = interface_->imuBuf_.front();
    //         double imuTime = imuMsg->header.stamp.sec + imuMsg->header.stamp.nanosec * 1e-9;

    //         if (imuTime < tIm)
    //         {
    //             cv::Point3f accel(
    //                 imuMsg->linear_acceleration.x,
    //                 imuMsg->linear_acceleration.y,
    //                 imuMsg->linear_acceleration.z
    //             );
    //             cv::Point3f gyro(
    //                 imuMsg->angular_velocity.x,
    //                 imuMsg->angular_velocity.y,
    //                 imuMsg->angular_velocity.z
    //             );
    //             vImuMeas.emplace_back(accel, gyro, imuTime);
    //             interface_->imuBuf_.pop();
    //         }
    //         else
    //         {
    //             break;
    //         }
    //     }

    //     return vImuMeas;
    // }

    

     
    
    void BaseSlamNode::publishMapPointCloud(std::shared_ptr<rmw_request_id_t> request_header,
                                            std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap::Request> request,
                                            std::shared_ptr<slam_msgs::srv::GetAllLandmarksInMap::Response> response)
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);

            if (mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            mapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_DEBUG_STREAM(this->get_logger(), "=======================");

            // Calculate the time taken for each line

            // Print the time taken for each line
            response->landmarks = mapPCL;
        }
    }

    void BaseSlamNode::resetActiveMapSrv(std::shared_ptr<rmw_request_id_t> request_header,
                           std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        interface_->resetLocalMapping();
    }

    void BaseSlamNode::publishMapData()
    {
        if (isTracked_)
        {
            auto start = std::chrono::high_resolution_clock::now();
            slam_msgs::msg::SlamInfo slamInfoMsg;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing map data");
            double tracking_freq = frequency_tracker_count_ / std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - frequency_tracker_clock_).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Current ORB-SLAM3 tracking frequency: " << tracking_freq << " frames / sec");
            frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
            frequency_tracker_count_ = 0;
            // publish the map data (current active keyframes etc)
            slam_msgs::msg::MapData mapDataMsg;
            interface_->mapDataToMsg(mapDataMsg, true, false);
            mapDataPub_->publish(mapDataMsg);
            slamInfoMsg.num_maps = interface_->getNumberOfMaps();
            slamInfoMsg.num_keyframes_in_current_map = mapDataMsg.graph.poses_id.size();
            slamInfoMsg.tracking_frequency = tracking_freq;
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_publishMapData = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapdata: " << time_publishMapData << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "*************************");
            slamInfoPub_->publish(slamInfoMsg);
        }
    }

    void BaseSlamNode::getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMap2 service called.");
        slam_msgs::msg::MapData mapDataMsg;
        interface_->mapDataToMsg(mapDataMsg, false, request->tracked_points, request->kf_id_for_landmarks);
        response->data = mapDataMsg;
    }

    void BaseSlamNode::getMapPointsInViewServer(std::shared_ptr<rmw_request_id_t> request_header,
                                                std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Request> request,
                                                std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMapPointsInView service called.");
        std::vector<slam_msgs::msg::MapPoint> landmarks;
        std::vector<ORB_SLAM3::MapPoint*> points;
        interface_->mapPointsVisibleFromPose(request->pose, points, 1000, request->max_dist_pose_observation, request->max_angle_pose_observation);
        auto affineMapToPos = interface_->getTypeConversionPtr()->poseToAffine(request->pose);
        auto affinePosToMap = affineMapToPos.inverse();
        // Populate the pose of the points vector into the ros message
        for (const auto& point : points) {
            slam_msgs::msg::MapPoint landmark;
            Eigen::Vector3f landmark_position = point->GetWorldPos();
            auto position = interface_->getTypeConversionPtr()->vector3fORBToROS(landmark_position);
            position = interface_->getTypeConversionPtr()->transformPointWithReference<Eigen::Vector3f>(affinePosToMap, position);
            // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << position.x() << " y: " << position.y() << " z: " << position.z());
            landmark.position.x = position.x();
            landmark.position.y = position.y();
            landmark.position.z = position.z();
            landmarks.push_back(landmark);
        }
        response->map_points = landmarks;
        auto cloud = interface_->getTypeConversionPtr()->MapPointsToPCL(points);
        visibleLandmarksPub_->publish(cloud);

        // Convert the pose in request to PoseStamped and publish
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map"; // Assuming the frame is "map", adjust if needed
        pose_stamped.pose = request->pose;

        // Publish the PoseStamped
        visibleLandmarksPose_->publish(pose_stamped);
    }

    // Member variable (add to class header)
    // std::deque<sensor_msgs::msg::Image::SharedPtr> rgb_buffer_;

    void BaseSlamNode::RGBCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB)
    {
        const double buffer_delay_sec = 0.75;  // 750ms delay buffer for IMU to catch up
        rgb_buffer_.push_back(msgRGB);
        // RCLCPP_INFO(this->get_logger(), "RGBCallback triggered! Time = %.3f", msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9);


        // Use ROS time
        rclcpp::Time ros_now = this->get_clock()->now();
        double now = ros_now.seconds();
        if (ros_now.seconds() == 0.0) {
            RCLCPP_WARN(this->get_logger(), "ROS time not initialized yet. Skipping image.");
            return;
        }

        while (!rgb_buffer_.empty())
        {
            const auto &img = rgb_buffer_.front();
            double ts = img->header.stamp.sec + img->header.stamp.nanosec * 1e-9;
            RCLCPP_INFO(this->get_logger(), "Processing image at t=%.6f, now = %.6f, now-ts=%.3f", ts, now, now - ts);
            if (now - ts >= buffer_delay_sec)
            {
                rgb_buffer_.pop_front();
                processRGBFrame(img);
            }
            else
            {
                break;
            }
        }
    }

    void BaseSlamNode::processRGBFrame(const sensor_msgs::msg::Image::SharedPtr &msgRGB)
    {
        double timestamp = msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec * 1e-9;
        if (!interface_->HasIMU()) {
            RCLCPP_WARN(this->get_logger(), "No IMU data yet, skipping image at t=%.6f", timestamp);
            return;
        }

        // Only check if we have IMU data at all
        if (!interface_->HasIMU()) {
            RCLCPP_WARN(this->get_logger(), "No IMU data yet, skipping image at t=%.6f", timestamp);
            return;
        }

        // Get the current time for log/debug
        double now = this->get_clock()->now().seconds();

        // Get IMU time BEFORE extraction
        double first_imu_time = interface_->GetFirstIMUMsgTime();
        RCLCPP_INFO(this->get_logger(), "Now: %.3f, Img: %.3f, First IMU: %.3f", now, timestamp, first_imu_time);

        // Skip if image timestamp is too early
        if (timestamp < first_imu_time) {
            RCLCPP_WARN(this->get_logger(),
                        "Image timestamp (%.6f) is before first IMU timestamp (%.6f). Skipping.",
                        timestamp, first_imu_time);
            return;
        }

        // Extract IMU measurements AFTER doing safety checks
        std::vector<ORB_SLAM3::IMU::Point> imuMeasurements = interface_->ExtractIMUUntil(timestamp);
        size_t imu_size = imuMeasurements.size();

        if (imu_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No IMU measurements for image t=%.6f. Skipping frame.", timestamp);
            RCLCPP_ERROR(this->get_logger(), "Extracted 0 IMU measurements. Aborting trackMonocular call.");
            return;
        }

        // Use extracted IMU, don't ask buffer again
        RCLCPP_WARN(this->get_logger(),
                    "Image t=%.6f, Extracted %zu IMU msgs from buffer", timestamp, imu_size);

        double lastImuTime = imuMeasurements.back().t;
        double dt = timestamp - lastImuTime;
        RCLCPP_DEBUG(this->get_logger(), "Δt (image - last IMU): %.6f", dt);

        Sophus::SE3f Tcw;
        try {
            bool success = interface_->trackMonocular(msgRGB, timestamp, imuMeasurements, Tcw);

            if (!success)
            {
                RCLCPP_WARN(this->get_logger(), "ORB-SLAM3 failed to track the frame at t=%.6f", timestamp);
                return;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in trackMonocular: %s", e.what());
            return;
        }

        isTracked_ = true;
        RCLCPP_INFO(this->get_logger(), "Tracking succeeded. isTracked_ = true");

        if (publish_tf_)
        {
            if (!odometry_mode_)
            {
                tfMapOdom_ = geometry_msgs::msg::TransformStamped();
                tfMapOdom_.header.stamp = msgRGB->header.stamp;
                tfMapOdom_.header.frame_id = global_frame_;
                tfMapOdom_.child_frame_id = odom_frame_id_;
                interface_->getDirectOdomToRobotTF(msgRGB->header, tfMapOdom_);
            }

            tfBroadcaster_->sendTransform(tfMapOdom_);
        }

        geometry_msgs::msg::PoseStamped pose;
        interface_->getRobotPose(pose);
        pose.header.stamp = msgRGB->header.stamp;
        robotPoseMapFrame_->publish(pose);

        ++frequency_tracker_count_;
    }


}
