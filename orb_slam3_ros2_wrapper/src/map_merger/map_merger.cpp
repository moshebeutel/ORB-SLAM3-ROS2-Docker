#include "map_merger.hpp"

MapMerger::MapMerger() : Node("map_merger")
{
    // Subscribers for robot map point clouds
    robot1_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot_0/map_points", 10, std::bind(&MapMerger::robot1MapCallback, this, std::placeholders::_1));
    robot2_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/robot_1/map_points", 10, std::bind(&MapMerger::robot2MapCallback, this, std::placeholders::_1));

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
    // Process robot1's map point cloud (store or use as needed)
    robot1_map_cloud_ = *msg; // store the cloud.
    // You would likely also call a function to pass this cloud to the orb slam instance.
    // orb_slam_interface_->addPointCloud(robot1_map_cloud_);
}

void MapMerger::robot2MapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Process robot2's map point cloud (store or use as needed)
    robot2_map_cloud_ = *msg; // store the cloud.
    // You would likely also call a function to pass this cloud to the orb slam instance.
    // orb_slam_interface_->addPointCloud(robot2_map_cloud_);
}

void MapMerger::checkMergeStatus()
{
    // Check if a merge is in progress
    if (orb_slam_interface_->getSLAMSystem()->GetLoopClosing()->mergeDetected())
    {
        RCLCPP_INFO(this->get_logger(), "Map merge in progress...");
        // Optionally, you can add logic to wait until the merge is complete.
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Checking merge status...");
    }

    // Publish the merged map data if the merge is complete.
    if (orb_slam_interface_->getSLAMSystem()->GetLoopClosing()->isFinished())
    {
        publishMergedMap();
    }
}

void MapMerger::publishMergedMap()
{
    {
        slam_msgs::msg::MapData merged_map_msg;
        // Temporarily comment out getAllKFIDs() until resolved
        orb_slam_interface_->mapDataToMsg(merged_map_msg, false, true /*, orb_slam_interface_->getAllKFIDs()*/);
        merged_map_pub_->publish(merged_map_msg);
        RCLCPP_INFO(this->get_logger(), "Merged map published.");
    }
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMerger>());
    rclcpp::shutdown();
    return 0;
}