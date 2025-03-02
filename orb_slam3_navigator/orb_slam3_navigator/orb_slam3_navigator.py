import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from slam_msgs.msg import MapData
from sensor_msgs.msg import LaserScan
import math

class ORBSLAMNavigator(Node):
    def __init__(self):
        super().__init__('orb_slam3_navigator')
        self.subscription = self.create_subscription(
            MapData,
            '/robot_0/map_data',
            self.map_data_callback,
            10)
        self.cmd_publisher = self.create_publisher(Twist, '/robot_0/cmd_vel', 10)
        self.current_pose = None
        self.stuck_counter = 0

    def map_data_callback(self, msg):
        self.get_logger().info("MapData Callback")
        # Extract the robot's position from the map data
        self.current_pose = msg.graph.poses_id[0]  # This assumes the first pose is the robot's position
        
        # Check for obstacles in the surrounding area within a 40 cm radius
        obstacles_detected = False
        for pose in msg.graph.poses_id:
            # Assuming that the poses_id contains the positions of obstacles or other robots
            distance = math.sqrt(pose.position.x**2 + pose.position.y**2)
            if distance < 0.4:
                obstacles_detected = True
                break
        
        if obstacles_detected:
            self.avoid_obstacle()
        else:
            self.move_forward()

    def avoid_obstacle(self):
        # Simple logic to avoid obstacles
        twist = Twist()
        # Check if the robot is too close to an obstacle, make a small turn
        twist.angular.z = 0.5  # Turn left or right (you can alternate this)
        self.cmd_publisher.publish(twist)
        self.get_logger().info('Obstacle detected, turning...')
        
        # Add some logic to ensure the robot doesn't get stuck
        self.check_if_stuck()

    def check_if_stuck(self):
        # If no progress for too long, reverse or move diagonally
        self.stuck_counter += 1
        if self.stuck_counter > 5:  # If stuck for too long, move backward
            twist = Twist()
            twist.linear.x = -0.1  # Move backward slightly
            self.cmd_publisher.publish(twist)
            self.get_logger().info('No clear path, moving backward...')
            self.stuck_counter = 0  # Reset counter after moving backward

    def move_forward(self):
        # Move forward if there are no obstacles
        twist = Twist()
        twist.linear.x = 0.2  # Move forward
        self.cmd_publisher.publish(twist)
        self.get_logger().info('Path clear, moving forward...')
        
        # Reset stuck counter when moving forward
        self.stuck_counter = 0

def main(args=None):
    rclpy.init(args=args)

    navigator = ORBSLAMNavigator()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

