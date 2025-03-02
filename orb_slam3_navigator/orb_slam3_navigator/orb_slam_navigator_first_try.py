import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import tf2_ros

class ORBSLAMNavigator(Node):
    def __init__(self):
        super().__init__('orb_slam_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.move)  # Control loop

        self.min_safe_distance = 0.2  # 20 cm threshold

    def move(self):
        twist = Twist()

        try:
            # Get transformation from ORB-SLAM3's map frame to camera frame
            trans = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

            z_depth = trans.transform.translation.z  # Distance in Z-axis

            self.get_logger().info(f'Current depth (Z): {z_depth:.2f}m')

            if z_depth > self.min_safe_distance:
                twist.linear.x = 0.2  # Move forward
            else:
                twist.angular.z = 1.0  # Turn left if too close

            self.cmd_pub.publish(twist)

        except tf2_ros.LookupException:
            self.get_logger().warn("TF transform not available yet!")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("TF data extrapolation error!")

def main(args=None):
    rclpy.init(args=args)
    navigator = ORBSLAMNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
