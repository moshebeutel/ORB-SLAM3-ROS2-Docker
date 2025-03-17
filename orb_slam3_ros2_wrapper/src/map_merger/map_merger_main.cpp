// src/map_merger/map_merger_main.cpp
#include "rclcpp/rclcpp.hpp"
#include "map_merger.hpp"
#include "ORB_SLAM3/Map.h"
#include "ORB_SLAM3/LoopClosing.h"
#include "ORB_SLAM3/Atlas.h" // Include Atlas header
#include "ORB_SLAM3/KeyFrameDatabase.h" //Include KeyFrameDatabase header.
#include "ORB_SLAM3/Vocabulary.h" //Include Vocabulary header.

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("map_merger_main"), "Map Merger node starting.");

    // Create shared instances of ORB-SLAM3 components
    auto global_map = std::make_shared<ORB_SLAM3::Map>();
    auto atlas = std::make_shared<ORB_SLAM3::Atlas>();
    auto keyFrameDatabase = std::make_shared<ORB_SLAM3::KeyFrameDatabase>();
    auto vocabulary = std::make_shared<ORB_SLAM3::Vocabulary>();

    // Instantiate LoopClosing with correct parameters
    try {
        auto loop_closer = std::make_shared<ORB_SLAM3::LoopClosing>(atlas.get(), keyFrameDatabase.get(), vocabulary.get(), true, true); // Adjust parameters as needed
        auto node = std::make_shared<ORB_SLAM3_Wrapper::MapMerger>(global_map, loop_closer, 10);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("map_merger_main"), "Error creating MapMerger node: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("map_merger_main"), "Map Merger node finished.");
    return 0;
}