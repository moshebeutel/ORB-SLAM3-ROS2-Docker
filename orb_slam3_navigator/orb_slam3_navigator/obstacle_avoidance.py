import rclpy
import struct
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Robot control publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/robot_0/cmd_vel", 10)

        # Subscribe to octomap point cloud
        self.create_subscription(PointCloud2, "/octomap_point_cloud_centers", self.process_point_cloud, 10)

        # Movement parameters
        self.forward_speed = 0.2  # Forward speed (m/s)
        self.turn_speed = 0.5  # Turning speed (rad/s)
        self.obstacle_distance_threshold = 0.35  # 35 cm
        self.turn_angle = np.radians(30)  # 30 degrees in radians

    def process_point_cloud(self, msg):
        # Extract point cloud data (x, y, z)
        points = self.parse_point_cloud(msg)

        # Filter points in front of the robot (y-axis)
        roi_points = [p for p in points if p[1] > 0 and abs(p[0]) < 0.5]  # Points in front and within 50cm width
        
        # Check if an obstacle is within 35 cm
        # obstacle_detected = any(p[1] < self.obstacle_distance_threshold for p in roi_points)
        # Count how many points are closer than 35 cm
        close_obstacle_count = sum(1 for p in roi_points if p[1] < self.obstacle_distance_threshold)
        self.get_logger().info(f"close_obstacle_count: {close_obstacle_count}")
        # Control logic
        twist = Twist()
        if close_obstacle_count >=20:
            self.get_logger().info("Obstacle detected! Turning left 30 degrees.")
            twist.angular.z = self.turn_speed
        else:
            self.get_logger().info("Path clear. Moving forward.")
            twist.linear.x = self.forward_speed

        # Publish command
        self.cmd_vel_pub.publish(twist)

    def parse_point_cloud(self, msg):
        """ Extracts (x, y, z) points from PointCloud2 message """
        points = []
        for i in range(msg.width):
            offset = i * msg.point_step
            x = struct.unpack_from("f", msg.data, offset)[0]
            y = struct.unpack_from("f", msg.data, offset + 4)[0]
            z = struct.unpack_from("f", msg.data, offset + 8)[0]
            points.append((x, y, z))
        return points

def main():
    rclpy.init()
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
