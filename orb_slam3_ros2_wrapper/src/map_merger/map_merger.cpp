// MapMerger.cpp (Implementation file)
#include "map_merger.hpp"

MapMerger::MapMerger(std::shared_ptr<ORB_SLAM3::Map> global_map, std::shared_ptr<ORB_SLAM3::LoopClosing> loop_closer)
    : global_map_(global_map), loop_closer_(loop_closer)
{
    node_ = std::make_shared<rclcpp::Node>("map_merger");
    
    // Subscribe dynamically to multiple robot map topics
    for (int i = 0; i < max_robots_; ++i)
    {
        std::string topic_name = "/robot_" + std::to_string(i) + "/map_points";
        auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&MapMerger::mapCallback, this, std::placeholders::_1, i));
        subscribers_.push_back(sub);
    }
}

void MapMerger::mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int robot_id)
{
    std::vector<ORB_SLAM3::MapPoint *> received_map_points = convertPointCloudToMapPoints(msg);
    mergeMaps(received_map_points, robot_id);
}

std::vector<ORB_SLAM3::MapPoint *> MapMerger::convertPointCloudToMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert PointCloud2 to ORB-SLAM3 MapPoints
    std::vector<ORB_SLAM3::MapPoint *> map_points;
    // Implement conversion logic here
    return map_points;
}

void MapMerger::mergeMaps(const std::vector<ORB_SLAM3::MapPoint *> &new_map_points, int robot_id)
{
    for (auto *mp : new_map_points)
    {
        if (isCoVisible(mp))
        {
            global_map_->AddMapPoint(mp);
        }
    }
    loop_closer_->InsertKeyFrame(global_map_->GetLastKeyFrame());
}

bool MapMerger::isCoVisible(ORB_SLAM3::MapPoint *mp)
{
    // Check if the map point is co-visible with existing keyframes
    for (auto *kf : global_map_->GetAllKeyFrames())
    {
        if (kf->isMapPointInFrame(mp))
        {
            return true;
        }
    }
    return false;
}
