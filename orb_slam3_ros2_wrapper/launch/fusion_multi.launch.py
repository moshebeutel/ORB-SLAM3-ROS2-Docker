from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Retrieve parameters from launch file or command line
    vocabulary_file_path = LaunchConfiguration('vocabulary_file_path', default='/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    config_file_path = LaunchConfiguration('config_file_path', default='/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml')

    orb_slam3_fusion_node = Node(
        package='orb_slam3_ros2_wrapper',
        executable='orb_slam3_multi_map_fusion',
        output='screen',
        parameters=[{'path_to_vocabulary_file': vocabulary_file_path}],
    )

    return LaunchDescription([orb_slam3_fusion_node])
