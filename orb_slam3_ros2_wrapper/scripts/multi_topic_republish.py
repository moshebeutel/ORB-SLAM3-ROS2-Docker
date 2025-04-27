import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your actual message type
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message

class MultiTopicRepublisher(Node):
    def __init__(self):
        super().__init__('multi_topic_republisher')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Define topic pairs: (input_topic, output_topic)
        self.topic_pairs = [
            ('/drn12345678/mavros/imu/data', Imu, '/robot_0/imu'),
            ('/active_drone_fpv', Image, '/robot_0/rgb_camera'),
            ('/drn12345678/mavros/global_position/local', Odometry, '/robot_0/ground_truth_pose'),
            # ('/active_drone_fpv', Image, '/robot_0/depth_camera'),
        ]

        # Dictionaries to store publishers
        self.my_publishers = {}

        # Create subscriber/publisher for each pair
        for input_topic, topic_type, output_topic in self.topic_pairs:
            self.create_subscription(
                topic_type,
                input_topic,
                self.make_callback(output_topic),
                qos
            )
            
            self.my_publishers[output_topic] = self.create_publisher(
                topic_type,
                output_topic,
                10
            )

    def make_callback(self, output_topic):
        def callback(msg):
            # self.get_logger().info(f'[{output_topic}] Republishing: "{msg}"')
            # if output_topic == '/robot_0/imu':
            #     # Deserialize the message if needed
            #     msg = Imu()
            # if output_topic == '/robot_0/ground_truth_pose':
            #     # Deserialize the message if needed
                # msg = Odometry()
            self.my_publishers[output_topic].publish(msg)
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
