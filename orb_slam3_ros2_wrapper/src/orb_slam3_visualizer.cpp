//
// Created by user1 on 28/05/25.
//
#include "orb_slam3_ros2_wrapper/orb_slam3_visualizer.hpp"

namespace ORB_SLAM3_Wrapper
{

    ORBSLAM3Visualizer::ORBSLAM3Visualizer(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<ORBSLAM3Interface> slam_interface)
        : node_(node), slam_interface_(slam_interface)
    {
        // Initialize publishers
        keyframes_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "orb_slam3/keyframes", 10);
        map_points_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "orb_slam3/map_points", 10);
        observations_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "orb_slam3/observations", 10);
    }

    void ORBSLAM3Visualizer::publishVisualizations()
    {
        publishKeyFrames();
        publishMapPoints();
        publishObservations();
    }

    void ORBSLAM3Visualizer::publishKeyFrames()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        // Implementation using slam_interface_ to get keyframe data
        keyframes_publisher_->publish(marker_array);
    }

    void ORBSLAM3Visualizer::publishMapPoints()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        slam_interface_->getAllMapPoints(cloud_msg);
        map_points_publisher_->publish(cloud_msg);
    }

    void ORBSLAM3Visualizer::publishObservations()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        // Implementation using slam_interface_ to get observation data
        observations_publisher_->publish(marker_array);
    }

} // namespace ORB_SLAM3_Wrapper