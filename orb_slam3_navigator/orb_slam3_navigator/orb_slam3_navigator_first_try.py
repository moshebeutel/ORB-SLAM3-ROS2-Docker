import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros

class ORBSLAMNavigator(Node):
    def __init__(self):
        super().__init__('orb_slam3_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/robot_0/cmd_vel', 10)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.timer = self.create_timer(0.1, self.move)  # Control loop
        # Subscribe to TF topic
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        
        self.min_safe_distance = 0.5  # 50 cm threshold
        self.last_turn_direction = 1  # 1 = left, -1 = right
        self.stuck_counter = 0  # To detect if we're stuck

    def tf_callback(self, msg):
        """
        Callback function triggered whenever a new TF message is received.
        We extract the relevant transformation and take action.
        """
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "robot_0/odom":
                try:
                    trans = self.tf_buffer.lookup_transform('map', 'robot_0/odom', rclpy.time.Time())
                    self.move(trans)
                except tf2_ros.LookupException:
                    self.get_logger().warn("TF transform not available yet!")
                except tf2_ros.ExtrapolationException:
                    self.get_logger().warn("TF data extrapolation error!")

    def move(self, trans):
        twist = Twist()

        try:
            z_depth = trans.transform.translation.z  # Distance in Z-axis

            self.get_logger().info(f'Current depth (Z): {z_depth:.2f}m')

            if z_depth > self.min_safe_distance:
                self.move_forward(linear=0.2)  # Move forward
                self.stuck_counter = 0  # Reset stuck counter
            else:
                self.stuck_counter += 1  # Increase stuck counter

                if self.stuck_counter > 3:  # If stuck multiple times
                    self.move_backward()  # Move back first
                else:
                    self.turn_robot()  # Turn to avoid obstacle

            

        except tf2_ros.LookupException:
            self.get_logger().warn("TF transform not available yet!")
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn("TF data extrapolation error!")

    def move_forward(self, linear=0.2):
        """
        Command the robot to move forward.
        """
        self.publish_velocity(linear=linear, angular=0.0)
        self.get_logger().info("Moving forward")

    def move_backward(self, linear=-0.1):
        """
        Command the robot to move backward slightly if stuck.
        """
        self.publish_velocity(linear=linear, angular=0.0)
        self.get_logger().info("Moving backward to avoid getting stuck")

    def turn_robot(self, angular=0.3):
        """
        Command the robot to turn in an alternating direction.
        """
        angular_velocity = self.last_turn_direction * angular
        self.publish_velocity(linear=0.0, angular=angular_velocity)

        # Alternate turn direction for next time
        self.last_turn_direction *= -1  

        self.get_logger().info(f"Turning {'left' if angular_velocity > 0 else 'right'}")

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = ORBSLAMNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
