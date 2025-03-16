// src/map_merger/map_merger_main.cpp
#include "rclcpp/rclcpp.hpp"
#include "map_merger.hpp"
#include "Map.h"
#include "LoopClosing.h"
#include "System.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("map_merger_main"), "Map Merger node starting.");

    try {
        const std::string &strVocFile,
        const std::string &strSettingsFile,
        ORB_SLAM3::System::eSensor sensor,
        bool bUseViewer,
        mSLAM_ = std::make_shared<ORB_SLAM3::System>(strVocFile_, strSettingsFile_, sensor_, bUseViewer_);
        auto global_map = mSLAM->GetAtlas()->GetCurrentMap(); // Retrieve the current map
        auto loop_closer = mSLAM->GetLoopClosing(); // Get the loop closure instance

        auto node = std::make_shared<ORB_SLAM3_Wrapper::MapMerger>(global_map, loop_closer, 10);


        // auto node = std::make_shared<ORB_SLAM3_Wrapper::MapMerger>(10); // Pass max_robots
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