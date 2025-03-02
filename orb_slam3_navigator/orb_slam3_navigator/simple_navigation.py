import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import struct
import math
import time


class SimpleNavigation(Node):

    def __init__(self):
        super().__init__('simple_navigation')
        self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.point_cloud_callback, 10)
        self.create_subscription(PoseStamped, '/robot_0/robot_pose_slam', self.pose_callback, 10)
        self.velocity_pub = self.create_publisher(Twist, '/robot_0/cmd_vel', 10)
        self.robot_radius = 0.2  # Adjust based on your robot's size
        self.focal_point = np.array([0.0, 0.0])
        self.point_cloud_data = []
        self.prev_focal_point = np.array([0.0, 0.0])
        self.stuck_timer = 0.0
        self.stuck_threshold = 0.5  # Time in seconds before considering the robot stuck
        self.is_turning = False
        self.turning_timer = 0.0
        self.turning_timeout = 1.0  # Time to turn before checking for a new direction
        self.min_free_space_distance = 0.2 # Minimum desired distance to obstacles
        self.stopped = False 
        self.fps = 26
        self.last_cmd_vel = Twist()

    def pose_callback(self, msg):
        self.focal_point = np.array([msg.pose.position.x, msg.pose.position.y])

    def point_cloud_callback(self, msg):
        # Control logic
        cmd_vel = Twist()
        
        if self.stopped:
            self.velocity_pub.publish(cmd_vel)
            self.last_cmd_vel = cmd_vel
            return 
        self.point_cloud_data = self.extract_point_cloud(msg)

        # Check if stuck
        if np.linalg.norm(self.focal_point - self.prev_focal_point) < 0.001:  # Adjust threshold as needed
            self.stuck_timer += (1.0 / self.fps)  # Calculate time based on FPS
            self.get_logger().info(f"Stuck time is {self.stuck_timer}.")
        else:
            self.stuck_timer = 0.0

        # Find the direction with the most free space
        best_direction = self.find_open_direction()
        self.get_logger().info(f"Best Direction is {best_direction}.")
        # Check if turning
        if abs(best_direction) > 1.8 and abs(best_direction)<=3.1:  # Adjust threshold for turning
            self.is_turning = True
            self.turning_timer += 0.1
        else:
            self.is_turning = False
            self.turning_timer = 0.0

        
        if self.stuck_timer > self.stuck_threshold:
            # Reverse if stuck
            cmd_vel.linear.x = -0.1  # Adjust reverse speed
            self.get_logger().info("Moving Reverse.")
        elif self.is_turning and self.turning_timer > self.turning_timeout:
            # If turning for too long, find a new direction
            self.is_turning = False
            best_direction = self.find_open_direction()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = best_direction * 0.1  # Adjust turning speed
            self.get_logger().info("Turning.")
        else:
            # Prioritize forward movement if the direction is within a small range
            if abs(best_direction) < 1.8 or abs(best_direction)>= 3.14: 
                cmd_vel.linear.x = 0.2  # Adjust forward speed
                cmd_vel.angular.z = 0.0
                self.get_logger().info("Moving forward.")
            else:
                cmd_vel.linear.x = 0.1  # Adjust forward speed during turning
                cmd_vel.angular.z = best_direction * 0.1  # Adjust turning speed
                self.get_logger().info("Forward with Turn.")

        self.velocity_pub.publish(cmd_vel)
        self.last_cmd_vel = cmd_vel

        self.prev_focal_point = np.copy(self.focal_point)


    def extract_point_cloud(self, msg):
        """ Convert PointCloud2 message to numpy array """
        pc_data = []
        
        # Extract data from PointCloud2 message (assuming 32-bit float)
        point_step = msg.point_step
        row_step = msg.row_step
        data = np.array(struct.unpack_from(f'{len(msg.data)//4}f', msg.data))
        
        # Reshape the data to 3D points (x, y, z)
        for i in range(0, len(data), 3):
            if i + 2 < len(data):  # Ensure we're within bounds
                pc_data.append([data[i], data[i+1], data[i+2]])
        
        return np.array(pc_data)

    def find_open_direction(self):
        """
        Find the direction with the most free space around the robot.

        Returns:
            float: Angle in radians (-pi to pi) indicating the direction to move.
        """
        num_directions = 36  # Number of directions to check
        angles = np.linspace(-np.pi, np.pi, num_directions)
        distances = []

        # Adjust min_free_space_distance based on robot speed
        dynamic_min_distance = self.min_free_space_distance + 0.1 * abs(self.last_cmd_vel.linear.x) 

        for angle in angles:
            direction_vector = np.array([math.cos(angle), math.sin(angle)])
            projected_distances = np.dot(self.point_cloud_data[:, :2] - self.focal_point, direction_vector) 
            forward_distances = projected_distances[projected_distances > 0] 

            if len(forward_distances) > 0:
                min_distance = np.min(forward_distances)
            else:
                min_distance = 0.0  # No obstacles in this direction

            # Penalize directions with very close obstacles
            if min_distance < dynamic_min_distance * 0.5: 
                min_distance *= 0.1  # Reduce the score for directions with close obstacles

            distances.append(min_distance)

        # Find the direction with the maximum distance
        best_direction_index = np.argmax(distances)
        best_direction = angles[best_direction_index]

        return best_direction

def main(args=None):
    rclpy.init(args=args)
    navigation_node = SimpleNavigation()

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        # Publish stop command before shutting down
        cmd_vel = Twist()
        navigation_node.velocity_pub.publish(cmd_vel)
        navigation_node.get_logger().info("Stopping navigation node...")
        navigation_node.stopped = True 
        # Allow some time for the robot to stop
        time.sleep(3.0) 

    # Shutdown only once
    if rclpy.ok():
        
        time.sleep(2)  # Allow some time for the command to be processed
        navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
