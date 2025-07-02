import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class ManualDriveNode(Node):
    def __init__(self):
        super().__init__("manual_drive")
        self.cmd_vel_pub = self.create_publisher(Twist, "/robot_0/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 sec
        self.step = 0

    def timer_callback(self):
        twist = Twist()
        # Sequence: forward, left, right, rotate, stop, repeat
        if np.mod(self.step, 50):
            self.get_logger().info("Moving forward")
            twist.linear.x = 0.2
            twist.angular.z = 0.1  # No rotation
        elif self.step == 10:
            self.get_logger().info("Turning left")
            twist.linear.x = 0.1
            twist.angular.z = 0.5
        elif self.step == 20:
            self.get_logger().info("Turning right")
            twist.linear.x = 0.1
            twist.angular.z = -0.5
        elif self.step == 30:
            self.get_logger().info("Rotating in place")
            twist.linear.x = 0.02  # Small forward motion to avoid stopping completely
            twist.angular.z = 1.0
        elif self.step == 40:
            self.get_logger().info("Stopping")
            # All zeros
        self.cmd_vel_pub.publish(twist)
        self.step = (self.step + 1) % 50  # Loop through steps

def main():
    rclpy.init()
    node = ManualDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()