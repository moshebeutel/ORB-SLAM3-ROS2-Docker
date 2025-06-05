#ifndef ORB_SLAM3_VISUALIZER_HPP
#define ORB_SLAM3_VISUALIZER_HPP

#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ORB_SLAM3_Wrapper
{

    class ORBSLAM3Visualizer
    {
    public:
        ORBSLAM3Visualizer(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<ORBSLAM3Interface> slam_interface);

        void publishVisualizations();

    private:
        void publishKeyFrames();
        void publishMapPoints();
        void publishObservations();

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<ORBSLAM3Interface> slam_interface_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframes_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr observations_publisher_;
    };

} // namespace ORB_SLAM3_Wrapper

#endif // ORB_SLAM3_VISUALIZER_HPP