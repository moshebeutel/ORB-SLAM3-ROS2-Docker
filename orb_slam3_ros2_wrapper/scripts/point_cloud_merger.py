import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PointCloudMerger(Node):

    def __init__(self):
        super().__init__('pointcloud_merger')

        self.declare_parameter('merge_distance_threshold', 5.0)  # Adjust as needed
        self.merge_distance_threshold = self.get_parameter('merge_distance_threshold').value

        self.declare_parameter('robot_0_start_x', 0.0)
        self.declare_parameter('robot_0_start_y', 0.0)
        self.declare_parameter('robot_1_start_x', 0.1)
        self.declare_parameter('robot_1_start_y', 0.0)

        self.robot_0_start_x = self.get_parameter('robot_0_start_x').value
        self.robot_0_start_y = self.get_parameter('robot_0_start_y').value
        self.robot_1_start_x = self.get_parameter('robot_1_start_x').value
        self.robot_1_start_y = self.get_parameter('robot_1_start_y').value

        self.robot_0_pose = None
        self.robot_1_pose = None

        self.robot_0_pc = None
        self.robot_1_pc = None

        self.robot_0_odom_sub = self.create_subscription(
            Odometry, '/robot_0/odom', self.robot_0_odom_callback, 10)
        self.robot_1_odom_sub = self.create_subscription(
            Odometry, '/robot_1/odom', self.robot_1_odom_callback, 10)

        self.robot_0_pc_sub = self.create_subscription(
            PointCloud2, '/robot_0/map_points', self.robot_0_pc_callback, 10)
        self.robot_1_pc_sub = self.create_subscription(
            PointCloud2, '/robot_1/map_points', self.robot_1_pc_callback, 10)

        self.merged_pc_pub = self.create_publisher(PointCloud2, '/merged_point_cloud', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def robot_0_odom_callback(self, msg: Odometry):
        self.robot_0_pose = msg.pose.pose

    def robot_1_odom_callback(self, msg: Odometry):
        self.robot_1_pose = msg.pose.pose

    def robot_0_pc_callback(self, msg: PointCloud2):
        self.robot_0_pc = msg
        self.attempt_merge()

    def robot_1_pc_callback(self, msg: PointCloud2):
        self.robot_1_pc = msg
        self.attempt_merge()

    def attempt_merge(self):
        if self.robot_0_pose is None or self.robot_1_pose is None or self.robot_0_pc is None or self.robot_1_pc is None:
            return

        robot_0_x = self.robot_0_pose.position.x
        robot_0_y = self.robot_0_pose.position.y
        robot_1_x = self.robot_1_pose.position.x
        robot_1_y = self.robot_1_pose.position.y

        distance = np.sqrt((robot_0_x - robot_1_x)**2 + (robot_0_y - robot_1_y)**2)

        start_distance = np.sqrt((self.robot_0_start_x - self.robot_1_start_x)**2 + (self.robot_0_start_y - self.robot_1_start_y)**2)

        if distance < self.merge_distance_threshold or start_distance < self.merge_distance_threshold:
            self.merge_pointclouds()

    def merge_pointclouds(self):
        try:
            transform_0_to_1 = self.tf_buffer.lookup_transform(
                self.robot_1_pc.header.frame_id,
                self.robot_0_pc.header.frame_id,
                rclpy.time.Time()
            )

            points_0 = pc2.read_points(self.robot_0_pc, field_names=("x", "y", "z"))
            points_1 = pc2.read_points(self.robot_1_pc, field_names=("x", "y", "z"))

            transformed_points_0 = []
            for p in points_0:
                p_transformed = self.transform_point(p, transform_0_to_1)
                transformed_points_0.append(p_transformed)

            merged_points = list(points_1) + transformed_points_0

            merged_pc = pc2.create_cloud_xyz32(self.robot_1_pc.header, merged_points)
            self.merged_pc_pub.publish(merged_pc)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform point cloud: {ex}')

    def transform_point(self, point, transform: TransformStamped):
        x = point[0]
        y = point[1]
        z = point[2]

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Quaternion rotation
        v = np.array([x, y, z])
        q = np.array([qx, qy, qz, qw])
        q_conj = np.array([-qx, -qy, -qz, qw])

        v_rotated = self.quaternion_rotate(q, v)
        v_transformed = v_rotated + np.array([tx, ty, tz])

        return v_transformed.tolist()

    def quaternion_rotate(self, q, v):
        q_v = np.array([v[0], v[1], v[2], 0])
        q_inv = np.array([-q[0], -q[1], -q[2], q[3]])
        q_v_rotated = self.quaternion_multiply(q, self.quaternion_multiply(q_v, q_inv))
        return q_v_rotated[:3]

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([x, y, z, w])

def main(args=None):
    rclpy.init(args=args)
    pointcloud_merger = PointCloudMerger()
    rclpy.spin(pointcloud_merger)
    pointcloud_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()