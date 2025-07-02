from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orb_slam3_ros2_wrapper',
            executable='mono',
            name='mono',
            namespace='/robot_0',
            parameters=[
                '/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml'
            ],
            arguments=[
                '/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                '/root/colcon_ws/src/orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml'
            ],
            prefix='gdb -ex run --args',
            output='screen',
        )
    ])
