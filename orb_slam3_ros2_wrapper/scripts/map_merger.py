import rclpy
from rclpy.node import Node
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree
import pcl

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')
        self.subscription_0 = self.create_subscription(PointCloud2, '/robot_0/map_points', self.callback_robot_0, 10)
        self.subscription_1 = self.create_subscription(PointCloud2, '/robot_1/map_points', self.callback_robot_1, 10)
        self.publisher = self.create_publisher(PointCloud2, '/merged_map_points', 10)
        self.robot_0_cloud = None
        self.robot_1_cloud = None

        # Declare dynamic parameters
        self.declare_parameter('distance_threshold', 0.5)  # Default: 50 cm
        self.declare_parameter('min_precentage_matching_points', 80)  # Default: 80%
        
        # Assume transformation (adjust as needed)
        self.translation = np.array([0.1, 0.0, 0.0])  # Example transformation
        self.rotation = np.eye(3)  # No rotation assumed

    def callback_robot_0(self, msg):
        self.robot_0_cloud = self.pointcloud_to_numpy(msg)
        self.try_merge()

    def callback_robot_1(self, msg):
        self.robot_1_cloud = self.pointcloud_to_numpy(msg)
        self.try_merge()

    def pointcloud_to_numpy(self, msg):
        print(f"Received PointCloud2 with width={msg.width}, height={msg.height}, is_dense={msg.is_dense}")

        if msg.width == 0 or msg.height == 0 or msg.data==0:
            print("Warning: Empty PointCloud2 received.")
            return np.array([])

        try:
            points = np.array([list(p) for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
            print(f"Converted PointCloud2 to numpy array with shape {points.shape}")
            return points
        except Exception as e:
            print(f"Error converting PointCloud2: {e}")
            return np.array([])

    def transform_robot_1_points(self):
        if self.robot_1_cloud.size==0:
            return None
        return (self.rotation @ self.robot_1_cloud.T).T + self.translation

    def try_merge(self):
        if self.robot_0_cloud is not None and self.robot_1_cloud is not None:
            # Get updated parameter values
            distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
            min_precentage_matching_points = self.get_parameter('min_precentage_matching_points').get_parameter_value().integer_value

            self.get_logger().info(f"Using distance threshold: {distance_threshold}m, min matching points: {min_precentage_matching_points}")

            # **Check if robot_1 sees robot_0**
            tree_0 = KDTree(self.robot_0_cloud)
            distances_0, _ = tree_0.query(self.robot_1_cloud, k=1)
            matches_0 = distances_0 < distance_threshold
            match_count_0 = np.sum(matches_0)

            # **Check if robot_0 sees robot_1**
            tree_1 = KDTree(self.robot_1_cloud)
            distances_1, _ = tree_1.query(self.robot_0_cloud, k=1)
            matches_1 = distances_1 < distance_threshold
            match_count_1 = np.sum(matches_1)

            # Calculate percentage of matched points
            percent_match_0 = (match_count_0 / len(self.robot_1_cloud)) * 100 if len(self.robot_1_cloud) > 0 else 0
            percent_match_1 = (match_count_1 / len(self.robot_0_cloud)) * 100 if len(self.robot_0_cloud) > 0 else 0

            self.get_logger().info(f"Robot_1 → Robot_0 match: {percent_match_0:.2f}%, Robot_0 → Robot_1 match: {percent_match_1:.2f}%")

            # Merge if both percentages exceed min_precentage_matching_points %
            if percent_match_0 >= min_precentage_matching_points and percent_match_1 >= min_precentage_matching_points:
                self.get_logger().info(f"Mutual visibility detected! Merging maps.")
                merged_cloud = np.vstack((self.robot_0_cloud, self.robot_1_cloud))
                self.get_logger().info(f"Merged point cloud shape: {merged_cloud.shape}")
            else:
                self.get_logger().warn(f"Insufficient mutual matches: {percent_match_0:.2f}% & {percent_match_1:.2f}%. Skipping merge.")

    def publish_merged_cloud(self, merged_cloud):
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        merged_msg = pc2.create_cloud_xyz32(header, merged_cloud)
        self.publisher.publish(merged_msg)
        self.get_logger().info("Published merged point cloud")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()