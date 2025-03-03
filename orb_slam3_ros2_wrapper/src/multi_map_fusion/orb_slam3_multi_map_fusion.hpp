/**
 * @file orb_slam3_multi_map_fusion.hpp
 * @brief Implementation of the ORBSLAM3MultiMapFusion class.
 */
#ifndef ORBSLAM3_MULTI_MAP_FUSION_HPP
#define ORBSLAM3_MULTI_MAP_FUSION_HPP

#include <iostream>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "System.h"
#include "Map.h"

namespace ORB_SLAM3_Wrapper
{
    class ORBSLAM3MultiMapFusion : public rclcpp::Node
    {
    public:
        ORBSLAM3MultiMapFusion();
        ~ORBSLAM3MultiMapFusion() = default;

        void processMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int robot_id);

    private:
        std::shared_ptr<ORB_SLAM3::System> slam_system_;
        std::unordered_map<int, std::vector<ORB_SLAM3::MapPoint *>> robot_maps_;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;

        std::string vocabulary_path_;
        double covisibility_threshold_;
        int num_robots_;

        std::vector<ORB_SLAM3::MapPoint *> convertToMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        double computeCovisibility(const std::vector<ORB_SLAM3::MapPoint *> &map1, const std::vector<ORB_SLAM3::MapPoint *> &map2);
        bool arePointsSimilar(ORB_SLAM3::MapPoint *pt1, ORB_SLAM3::MapPoint *pt2);
    };
}

#endif // ORBSLAM3_MULTI_MAP_FUSION_HPP
