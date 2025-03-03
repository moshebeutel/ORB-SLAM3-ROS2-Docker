#include "rclcpp/rclcpp.hpp"
#include "orb_slam3_multi_map_fusion.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3MultiMapFusion>(); // Corrected class name
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}