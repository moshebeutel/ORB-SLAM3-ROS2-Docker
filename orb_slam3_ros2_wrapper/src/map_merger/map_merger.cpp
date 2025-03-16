// map_merger.cpp (Implementation file)
#include "map_merger.hpp"

namespace ORB_SLAM3_Wrapper {

MapMerger(std::shared_ptr<ORB_SLAM3::Map> global_map, std::shared_ptr<ORB_SLAM3::LoopClosing> loop_closer, int max_robots);

    : Node("map_merger_node"), max_robots_(max_robots), tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {

    RCLCPP_INFO(this->get_logger(), "MapMerger node initialized.");

    // Declare and retrieve vocabulary path parameter
    this->declare_parameter<std::string>("vocabulary_path", ""); // Default value is empty
    this->get_parameter("vocabulary_path", vocabulary_path_);

    // Initialize ORB-SLAM3 components
    global_map_ = std::make_shared<ORB_SLAM3::Map>();
    atlas_ = std::make_shared<ORB_SLAM3::Atlas>();
    keyFrameDatabase_ = std::make_shared<ORB_SLAM3::KeyFrameDatabase>();

    // Load vocabulary
    vocabulary_ = std::make_shared<ORB_SLAM3::Vocabulary>();
    if (!vocabulary_path_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loading vocabulary from: %s", vocabulary_path_.c_str());
        vocabulary_->load(vocabulary_path_); // Load the vocabulary
    } else {
        RCLCPP_WARN(this->get_logger(), "Vocabulary path not provided. Loop closure might not work.");
    }

    loop_closer_ = std::make_shared<ORB_SLAM3::LoopClosing>(atlas_.get(), keyFrameDatabase_.get(), vocabulary_.get(), true, true);

    // Subscribe to topics
    for (int i = 0; i < max_robots_; ++i) {
        std::string topic_name = "/robot_" + std::to_string(i) + "/map_points";
        auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
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
}
