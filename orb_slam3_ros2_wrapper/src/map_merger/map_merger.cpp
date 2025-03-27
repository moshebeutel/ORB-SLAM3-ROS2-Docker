#include "map_merger.hpp"

MapMerger::MapMerger() : Node("map_merger")
{
    // Subscribers for robot map point clouds
    robot1_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot_0/map_points", 10, std::bind(&MapMerger::robot1MapCallback, this, std::placeholders::_1));
    robot2_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot_1/map_points", 10, std::bind(&MapMerger::robot2MapCallback, this, std::placeholders::_1));

    // Odometry subscription for Robot 1
    robot1_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot_0/ground_truth_pose", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            RCLCPP_INFO(this->get_logger(), "Odometry callback triggered for Robot 1.");

            // Update Robot 0's pose from the odometry message
            robot1_pose_.pose = odom_msg->pose.pose;

            RCLCPP_INFO(this->get_logger(), "Robot 1 pose updated from odometry: [%.2f, %.2f, %.2f]",
                        robot1_pose_.pose.position.x, robot1_pose_.pose.position.y, robot1_pose_.pose.position.z);
        });

    // Odometry subscription for Robot 2
    robot2_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot_1/ground_truth_pose", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            RCLCPP_INFO(this->get_logger(), "Odometry callback triggered for Robot 2.");

            // Update Robot 2's pose from the odometry message
            robot2_pose_.pose = odom_msg->pose.pose;

            RCLCPP_INFO(this->get_logger(), "Robot 2 pose updated from odometry: [%.2f, %.2f, %.2f]",
                        robot2_pose_.pose.position.x, robot2_pose_.pose.position.y, robot2_pose_.pose.position.z);
        });

    RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_0/ground_truth_pose for Robot 1 odometry.");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /robot_1/ground_truth_pose for Robot 2 odometry.");

    // Publisher for merged map data
    merged_map_pub_ = this->create_publisher<slam_msgs::msg::MapData>("/merged_map", 10);

    // ORB-SLAM3 interface instance
    orb_slam_interface_ = std::make_unique<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(
        "/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt",       // strVocFile
        "/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml",         // strSettingsFile
        ORB_SLAM3::System::MONOCULAR,     // sensor (example: MONOCULAR, STEREO, RGBD)
        true,                             // bUseViewer
        0.0,                              // robotX
        0.0,                              // robotY
        "map",                            // globalFrame
        "odom",                           // odomFrame
        "base_link"                       // robotFrame
    );

    // Timer to periodically check merge status
    merge_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMerger::checkMergeStatus, this));
}

void MapMerger::robot1MapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Store Robot 1's map point cloud
    robot1_map_cloud_ = *msg;

    // Retrieve and store Robot 1's current pose
    orb_slam_interface_->getRobotPose(robot1_pose_);

    RCLCPP_INFO(this->get_logger(), "Robot 1 map point cloud processed. Pose: [%.2f, %.2f, %.2f]",
                robot1_pose_.pose.position.x, robot1_pose_.pose.position.y, robot1_pose_.pose.position.z);
}

void MapMerger::robot2MapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Store Robot 2's map point cloud
    robot2_map_cloud_ = *msg;

    RCLCPP_INFO(this->get_logger(), "Robot 2 map point cloud processed.");
}

void MapMerger::checkMergeStatus()
{
    // Check if ORB-SLAM3 has detected a loop closure or merge
    if (orb_slam_interface_->getSLAMSystem()->GetLoopClosing()->mergeDetected())
    {
        RCLCPP_INFO(this->get_logger(), "Map merge detected by ORB-SLAM3. Waiting for completion...");
    }

    // Check if the merge process is finished
    if (orb_slam_interface_->getSLAMSystem()->GetLoopClosing()->isFinished())
    {
        // Calculate the distance between the robots
        double distance = std::sqrt(std::pow(robot1_pose_.pose.position.x - robot2_pose_.pose.position.x, 2) +
                                    std::pow(robot1_pose_.pose.position.y - robot2_pose_.pose.position.y, 2) +
                                    std::pow(robot1_pose_.pose.position.z - robot2_pose_.pose.position.z, 2));

        // Define a threshold for merging (e.g., 5 meters)
        const double merge_threshold = 5.0;

        if (distance < merge_threshold)
        {
            RCLCPP_INFO(this->get_logger(), "Robots are close enough to merge. Distance: %.2f meters", distance);
            publishMergedMap();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Robots are too far apart to merge. Distance: %.2f meters", distance);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No merge detected. Continuing to monitor...");
    }
}

void MapMerger::publishMergedMap()
{
    slam_msgs::msg::MapData merged_map_msg;

    // Convert ORB-SLAM3's internal map data to a ROS message
    orb_slam_interface_->mapDataToMsg(merged_map_msg, false, true);

    // Publish the merged map
    merged_map_pub_->publish(merged_map_msg);

    RCLCPP_INFO(this->get_logger(), "Merged map published successfully.");
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMerger>());
    rclcpp::shutdown();
    return 0;
}