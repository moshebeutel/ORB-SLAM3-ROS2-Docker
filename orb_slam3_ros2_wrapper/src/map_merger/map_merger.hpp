// MapMerger.h
#ifndef MAPMERGER_H
#define MAPMERGER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ORB_SLAM3/LoopClosing.h"
#include "ORB_SLAM3/Map.h"
#include "ORB_SLAM3/MapPoint.h"
#include "ORB_SLAM3/Atlas.h"
#include "ORB_SLAM3/KeyFrameDatabase.h"
#include "ORB_SLAM3/Vocabulary.h"
#include <vector>
#include <memory>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ORB_SLAM3_Wrapper {

class MapMerger : public rclcpp::Node {
public:
    MapMerger(std::shared_ptr<ORB_SLAM3::Map> global_map, std::shared_ptr<ORB_SLAM3::LoopClosing> loop_closer, int max_robots);
    ~MapMerger();

    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int robot_id);
    std::vector<ORB_SLAM3::MapPoint *> convertPointCloudToMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void mergeMaps(const std::vector<ORB_SLAM3::MapPoint *> &new_map_points, int robot_id);
    bool isCoVisible(ORB_SLAM3::MapPoint *mp);

private:
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_; // Corrected line
    std::shared_ptr<ORB_SLAM3::Map> global_map_;
    std::shared_ptr<ORB_SLAM3::LoopClosing> loop_closer_;
    int max_robots_;
    std::string global_frame_ = "map";
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<ORB_SLAM3::Atlas> atlas_;
    std::shared_ptr<ORB_SLAM3::KeyFrameDatabase> keyFrameDatabase_;
    std::shared_ptr<ORB_SLAM3::Vocabulary> vocabulary_;
    std::string vocabulary_path_;
    double covisibility_threshold_;
    int num_robots_;
};

} // namespace ORB_SLAM3_Wrapper

#endif // MAPMERGER_H