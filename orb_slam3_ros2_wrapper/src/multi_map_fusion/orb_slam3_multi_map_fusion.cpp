#include "orb_slam3_multi_map_fusion.hpp"

namespace ORB_SLAM3_Wrapper {

    ORBSLAM3MultiMapFusion::ORBSLAM3MultiMapFusion() : rclcpp::Node("multi_map_fusion_node") {
        // ... constructor body ...
        // Declare parameters
        this->declare_parameter<std::string>("path_to_vocabulary_file", "");
        this->declare_parameter<double>("covisibility_threshold", 0.3);
        this->declare_parameter<int>("num_robots", 2);
    
        // Get parameter values from launch file
        this->get_parameter("path_to_vocabulary_file", vocabulary_path_);
        this->get_parameter("covisibility_threshold", covisibility_threshold_);
        this->get_parameter("num_robots", num_robots_);
    
        // Validate parameters
        if (vocabulary_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path to vocabulary file is required!");
            return;
        }
    
        // Initialize ORB-SLAM3 system
        slam_system_ = std::make_shared<ORB_SLAM3::System>(
            vocabulary_path_, "path_to_config_file", ORB_SLAM3::System::MONOCULAR, true);
    
        // Create subscriptions for N robots
        for (int i = 0; i < num_robots_; ++i)
        {
            std::string topic_name = "robot_" + std::to_string(i) + "/map_points";
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_name, 10,
                [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    processMapPoints(msg, i);
                });
            subscriptions_.push_back(sub);
        }
    }
    
     // namespace ORB_SLAM3_Wrapper

     void ORBSLAM3MultiMapFusion::processMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int robot_id) {
        std::vector<ORB_SLAM3::MapPoint *> map_points = convertToMapPoints(msg);
        robot_maps_[robot_id] = map_points;
    
        for (auto &[other_robot_id, other_map] : robot_maps_) {
            if (other_robot_id != robot_id) {
                double covisibility = computeCovisibility(map_points, other_map);
                if (covisibility >= covisibility_threshold_) {
                    ORB_SLAM3::Map* map = slam_system_->GetAtlas()->GetActiveMap();
                    if (map) {
                        map->MergeMaps(map_points, other_map);
                        map->DetectLoopClosures();
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "ORB-SLAM3 Map is null!");
                    }
                }
            }
        }
    }

    std::vector<ORB_SLAM3::MapPoint *> ORBSLAM3MultiMapFusion::convertToMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<ORB_SLAM3::MapPoint *> map_points;
        return map_points;
    }

    double ORBSLAM3MultiMapFusion::computeCovisibility(const std::vector<ORB_SLAM3::MapPoint *> &map1, const std::vector<ORB_SLAM3::MapPoint *> &map2)
    {
        int shared_count = 0;
        for (const auto &pt1 : map1)
        {
            for (const auto &pt2 : map2)
            {
                if (arePointsSimilar(pt1, pt2))
                {
                    shared_count++;
                }
            }
        }
        return static_cast<double>(shared_count) / std::min(map1.size(), map2.size());
    }

    bool ORBSLAM3MultiMapFusion::arePointsSimilar(ORB_SLAM3::MapPoint *pt1, ORB_SLAM3::MapPoint *pt2)
    {
        return (pt1->GetWorldPos() - pt2->GetWorldPos()).norm() < 0.05;
    }
}