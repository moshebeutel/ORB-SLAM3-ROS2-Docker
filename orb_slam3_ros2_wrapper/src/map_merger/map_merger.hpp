#ifndef MAP_MERGER_H
#define MAP_MERGER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "slam_msgs/msg/map_data.hpp"
#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"

class MapMerger : public rclcpp::Node
{
public:
    MapMerger(); // Constructor
    std::shared_ptr<ORB_SLAM3_Wrapper::ORBSLAM3Interface> getSLAMSystem();

private:
    void robot1MapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void robot2MapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void checkMergeStatus();
    void publishMergedMap();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot1_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr robot2_map_sub_;
    rclcpp::Publisher<slam_msgs::msg::MapData>::SharedPtr merged_map_pub_;
    rclcpp::TimerBase::SharedPtr merge_timer_;
    sensor_msgs::msg::PointCloud2 robot1_map_cloud_;
    sensor_msgs::msg::PointCloud2 robot2_map_cloud_;
    std::unique_ptr<ORB_SLAM3_Wrapper::ORBSLAM3Interface> orb_slam_interface_;

};

#endif // MAP_MERGER_H