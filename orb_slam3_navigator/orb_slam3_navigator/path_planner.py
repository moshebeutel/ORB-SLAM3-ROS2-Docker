import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Subscribers and publishers
        self.subscription = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.listener_callback,
            10
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot_0/robot_pose_slam',  # Adjust this topic based on your setup
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/robot_0/cmd_vel', 10)

        # Parameters for bounding box and navigation
        self.robot_length = 0.4  # size of the bounding box cube
        self.focal_point = np.array([0.0, 0.0, 0.0])  # Initialize robot focal point
        self.goal_position = np.array([0.0, 0.0, 0.0])  # Will update with the next best goal
        self.point_cloud_data = []
        self.max_turns = 3  # Limit number of turns per cycle
        self.turn_counter = 0  # Track the number of turns

        self.last_position = np.array([0.0, 0.0, 0.0])  # Track the last position of the robot
        self.stuck_threshold = 0.4  # Threshold distance to consider the robot stuck
        self.reverse_duration = 1.0  # Time to reverse (in seconds)
        self.time_stuck = 0  # Track time spent stuck
        self.reverse_mode = False  # Flag for reverse mode


    def pose_callback(self, msg):
        # Get the robot's position from the Odometry message
        self.focal_point = np.array([msg.pose.position.x, 
                                     msg.pose.position.y, 
                                     msg.pose.position.z])

    def listener_callback(self, msg):
        # Extract point cloud data
        self.point_cloud_data = self.extract_point_cloud(msg)       
        # Check if robot is stuck
        self.check_if_stuck()

        # Find the neighboring cubes based on the focal point
        neighboring_cubes = self.get_neighboring_cubes(self.focal_point)
        
        # Find the best cube to move to
        self.goal_position = self.find_best_cube(neighboring_cubes)
        
        # Send the velocity command to move towards the goal
        self.move_towards_goal(self.goal_position)

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

    def get_neighboring_cubes(self, focal_point):
        """ Generate neighboring cubes coordinates (26 neighbors excluding self) """
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if not (i == 0 and j == 0 and k == 0):  # Exclude the focal point itself
                        neighbor = np.array([focal_point[0] + i * self.robot_length,
                                             focal_point[1] + j * self.robot_length,
                                             focal_point[2] + k * self.robot_length])
                        neighbors.append(neighbor)
        return neighbors

    def find_best_cube(self, neighboring_cubes):
        """ Calculate and find the best free cube to move towards """
        best_cube = None
        min_obstacle_density = float('inf')
        max_proximity_weight = float('-inf')  # Initialize to track highest proximity weight
    
        for cube in neighboring_cubes:
            obstacle_density = self.calculate_obstacle_density(cube)
            proximity_weight = self.calculate_proximity_weight(cube)
            # If the cube is clear of obstacles, consider it for movement
            if obstacle_density == 0:  # Free cube, no obstacles
                # Use both proximity weight and obstacle density to find the best cube
                total_weight = proximity_weight  # You could combine this with obstacle density if needed
                if total_weight > max_proximity_weight:
                    max_proximity_weight = total_weight
                    best_cube = cube
    
        return best_cube if best_cube is not None else self.focal_point


    def calculate_obstacle_density(self, cube):
        """ Calculate obstacle density in the given cube """
        count = 0
        for point in self.point_cloud_data:
            if (cube[0] - self.robot_length <= point[0] <= cube[0] + self.robot_length and
                cube[1] - self.robot_length <= point[1] <= cube[1] + self.robot_length and
                cube[2] - self.robot_length <= point[2] <= cube[2] + self.robot_length):
                count += 1
        return count
    
    def calculate_proximity_weight(self, cube, max_distance=2.0):
        """Calculate proximity-based weight for a cube based on its distance from the robot."""
        total_weight = 0
        for point in self.point_cloud_data:
            distance = np.linalg.norm(point[:2] - cube[:2])  # Only use x, y for distance
            if distance < max_distance:
                total_weight += (1 - distance / max_distance)  # Closer points weigh more
        return total_weight


    def move_towards_goal(self, goal_position):
        """ Send velocity command to move towards the goal position """
        twist = Twist()
    
        # Calculate movement direction
        direction_vector = goal_position - self.focal_point
        distance = np.linalg.norm(direction_vector[:2])  # Only consider (x, y)
    
        # Check if moving forward is possible
        if self.is_direction_open(goal_position[:2]):  # Ignore z-axis
            twist.linear.x = min(0.5, distance)  # Move forward but cap speed
            twist.angular.z = 0.0  # No turning
            self.get_logger().info("Moving forward.")
        else:
            # Turning logic only if forward is blocked
            angle = np.arctan2(direction_vector[1], direction_vector[0])
            twist.linear.x = 0.0  # Stop forward motion
            twist.angular.z = np.clip(angle, -np.radians(30), np.radians(30))  # Limit turn to +/- 30 degrees
            self.get_logger().info("Turning to adjust direction.")

        self.publisher.publish(twist)



    def check_if_stuck(self):
        """ Detect if the robot is stuck and find a new path """
        distance_moved = np.linalg.norm(self.focal_point - self.last_position)
        current_time = self.get_clock().now().to_msg().sec
        self.get_logger().info(f"Distance moved = {distance_moved}")
        if 0.01 < distance_moved < self.stuck_threshold:
            if not self.reverse_mode:
                self.time_stuck_start = current_time
                self.reverse_mode = True
                self.last_failed_direction = self.goal_position  # Save dead-end
                self.get_logger().info("Robot is stuck! Reversing...")
    
        if self.reverse_mode:
            if current_time - self.time_stuck_start >= self.reverse_duration:
                self.reverse_mode = False
                self.get_logger().info("Finished reversing. Finding a new path.")
                self.find_new_direction()  # Find a new way forward

        self.last_position = self.focal_point

    def find_new_direction(self):
        """ Choose a new direction that avoids the last dead-end """
        angles = [30, -30, 45, -45, 15, -15]  # More directional changes
        for angle in angles:
            new_goal = self.rotate_vector(self.focal_point, angle)
            if self.is_direction_open(new_goal) and not np.allclose(new_goal[:2], self.last_failed_direction[:2], atol=0.1):
                self.goal_position = new_goal
                self.get_logger().info(f"New direction found at {angle} degrees! Moving.")
                return
        self.get_logger().info("No open direction found! Stopping.")


    def rotate_vector(self, vector, angle_degrees):
        """ Rotate a 2D vector by a given angle (degrees) """
        angle_radians = np.radians(angle_degrees)
        rotation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]
    ])
        return np.dot(rotation_matrix, vector[:2])  # Apply rotation in 2D

    def is_direction_open(self, new_direction):
        """ Check if the new direction is obstacle-free """
        for point in self.point_cloud_data:
            if np.linalg.norm(point[:2] - new_direction) < self.robot_length:
                return False  # Obstacle detected
        return True  # Path is clear





def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

