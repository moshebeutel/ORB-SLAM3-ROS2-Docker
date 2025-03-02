import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import math

import threading


class ProjectedMapNavigator(Node):
    def __init__(self):
        super().__init__('projected_map_navigator')
        
        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/projected_map', self.map_callback, 1000)
        
        self.subscription_pose = self.create_subscription(
            PoseStamped, '/robot_0/robot_pose_slam', self.pose_callback, 10)
        
        self.publisher_cmd_vel = self.create_publisher(
            Twist, '/robot_0/cmd_vel', 100)
        
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.robot_pose = None
        self.last_map_update_time = time.time()
        self.last_position = None
        self.robot_stuck = False

        self.is_in_recovery = False  # Flag to track recovery state
        self.reverse_timer = None
        self.rotation_timer = None
        self.recovery_completed = True  # Add this flag to track recovery state


        self.map_lock = threading.Lock()
        self.pose_lock = threading.Lock()
    
    def map_callback(self, msg):
        """Callback function for map data."""
        if self.is_in_recovery:
            self.get_logger().info("In recovery mode, ignoring map callback.")
            return  # Ignore map updates while in recovery mode
        # Acquire the lock before modifying shared data
        with self.map_lock:
            self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.map_resolution = msg.info.resolution
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Reset map update timer
        self.last_map_update_time = time.time()
        self.get_logger().info('Map received.')
    
    def pose_callback(self, msg):
        with self.pose_lock:
            self.robot_pose = msg.pose
            self.get_logger().info(f'Robot position: {self.robot_pose.position.x}, {self.robot_pose.position.y}')
            self.navigate()
            if self.check_for_stuck_robot():
                self.robot_stuck = True
                self.recovery_action()

    
    def check_for_stuck_robot(self):
        """Check if the robot has moved enough in the last 2 seconds."""
        if self.last_position is None:
            self.last_position = self.robot_pose.position
            return False

        # Calculate the distance the robot has moved
        dx = self.robot_pose.position.x - self.last_position.x
        dy = self.robot_pose.position.y - self.last_position.y
        distance_moved = math.sqrt(dx**2 + dy**2)

        self.last_position = self.robot_pose.position
        
        if distance_moved < 0.00005:  # Robot hasn't moved much (less than 5 cm)
            self.get_logger().info("Robot might be stuck.")
            return True
        return False


    
    def world_to_map(self, x, y):
        if self.map_data is None:
            return None, None
        
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        
        if 0 <= mx < self.map_data.shape[1] and 0 <= my < self.map_data.shape[0]:
            return mx, my
        return None, None
    
    def get_free_distance(self, map_x, map_y, direction):
        """Measure how far the robot can move in a given direction before hitting an obstacle."""
        dx, dy = direction  # Direction vectors (e.g., forward = (1, 0))

        distance = 0
        while (0 <= map_x < self.map_width and 0 <= map_y < self.map_height and
            self.map_data[map_y, map_x] == 0):  # Free space
            map_x += dx
            map_y += dy
            distance += 1

        return distance

    
    def find_best_direction(self, map_x, map_y, yaw):
        """Determine the best direction based on open space."""
        # Ensure map_x and map_y are within the bounds of the map
        if not (0 <= map_x < self.map_width and 0 <= map_y < self.map_height):
            self.get_logger().warn(f"Invalid map coordinates: ({map_x}, {map_y})")
            return "forward"  # Default to moving forward if out of bounds

        self.get_logger().info(f"Map coordinates: ({map_x}, {map_y})")

        # Define movement vectors for forward, left, right
        directions = {
            "forward": (int(np.cos(yaw)), int(np.sin(yaw))),
            "left": (int(np.cos(yaw + np.pi / 2)), int(np.sin(yaw + np.pi / 2))),
            "right": (int(np.cos(yaw - np.pi / 2)), int(np.sin(yaw - np.pi / 2)))
        }

        # Measure free space in each direction
        distances = {dir_name: self.get_free_distance(map_x, map_y, dir_vec)
                    for dir_name, dir_vec in directions.items()}

        # Log distances
        self.get_logger().info(f"Distances: {distances}")

        # Prioritize moving forward, otherwise turn
        best_direction = max(distances, key=distances.get)
        return best_direction    


    def move_robot(self, best_direction):
        """Send movement commands based on the best direction."""
        twist = Twist()
        self.get_logger().info(f'Navigating {best_direction}...')
        if best_direction == "forward":
            twist.linear.x = 0.2  # Move forward
        elif best_direction == "left":
            twist.angular.z = 0.1  # Turn left
        elif best_direction == "right":
            twist.angular.z = -0.1  # Turn right
        
        self.publisher_cmd_vel.publish(twist)

    
    def is_free(self, mx, my):
        if self.map_data is None:
            return False
        return self.map_data[my, mx] == 0


    def get_robot_yaw(self):
        """Extract the yaw angle (rotation around Z) from the robot's pose."""
        q = self.robot_pose.orientation  # Quaternion from PoseStamped

        # Compute yaw (rotation around Z-axis)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        return yaw  # Returns yaw in radians (-π to π)

    
    def navigate(self):
        if self.is_in_recovery:
            # reverse movement
            twist = Twist()
            twist.linear.x = -0.1  # Move backward slightly
            self.publisher_cmd_vel.publish(twist)
            self.get_logger().info("Skipping navigation since robot is in recovery.")
            return  # Skip navigation if the robot is in recovery mode

        if self.map_data is None or self.robot_pose is None:
            return

        # Convert robot position to map coordinates
        map_x, map_y = self.world_to_map(self.robot_pose.position.x, self.robot_pose.position.y)

        # Get the best direction to move
        yaw = self.get_robot_yaw()  # Implement yaw extraction
        best_direction = self.find_best_direction(map_x, map_y, yaw)

        # Move in the best direction
        self.move_robot(best_direction)

    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    
    def is_map_stagnant(self):
        """Check if the map has not been updated for too long."""
        return (time.time() - self.last_map_update_time) > 2  # 2 seconds timeout 

    
    def recovery_action(self):
        """Attempt to recover robot if it is stuck or map is outdated."""
        if self.robot_stuck:
            self.get_logger().info("Robot is stuck, attempting recovery...")
            self.reverse_robot()  # Reverse the robot a bit
            time.sleep(1)  # Pause before rotating
            self.rotate_robot()  # Rotate robot to find a path
        elif self.is_map_stagnant():
            self.get_logger().info("Map is stagnant, attempting recovery...")
            self.rotate_robot()  # Rotate to find new information

        # Move robot forward slowly to recheck map updates
        self.move_robot('forward')
        time.sleep(1)  # Allow some time to observe the environment

    
    def reverse_robot(self):
        """Move the robot backward a little to avoid being stuck."""
        if self.is_in_recovery or not self.recovery_completed:
            # reverse movement
            twist = Twist()
            twist.linear.x = -0.1  # Move backward slightly
            self.publisher_cmd_vel.publish(twist)
            return  # Avoid reversing if already in recovery or recovery not completed

        self.is_in_recovery = True  # Start recovery mode
        self.recovery_completed = False  # Mark recovery as in progress
        self.get_logger().info("Reversing robot.")
        
        # Start the reverse movement
        twist = Twist()
        twist.linear.x = -0.1  # Move backward slightly
        self.publisher_cmd_vel.publish(twist)

        # Start a timer to stop the reverse movement after a duration
        if self.reverse_timer is not None:
            self.reverse_timer.cancel()  # Cancel any previous reverse timer

        self.reverse_timer = self.create_timer(3.0, self.stop_reverse_robot)
        self.get_logger().info("Reverse movement started for 3 seconds.")


    def stop_reverse_robot(self):
        """Stop the robot after the reverse duration."""
        twist = Twist()
        twist.linear.x = 0.0  # Stop the robot after reversing
        self.publisher_cmd_vel.publish(twist)

        # Stop the timer after the reverse action is complete
        if self.reverse_timer is not None:
            self.reverse_timer.cancel()
        self.reverse_timer = None

        self.get_logger().info("Stopping robot after reverse.")
        
        # Rotate after reversing to avoid going back into the same obstacle
        self.rotate_robot()

    
    def rotate_robot(self):
        """Rotate the robot slightly to avoid going back into the same obstacle."""
        twist = Twist()
        twist.angular.z = 0.3  # Rotate a little
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info("Rotating robot to avoid re-hitting the obstacle.")
        
        # Stop rotation after a short time
        self.rotation_timer = self.create_timer(1.0, self.stop_rotation)

    def stop_rotation(self):
        """Stop the rotation after a short duration."""
        twist = Twist()
        twist.angular.z = 0.0  # Stop rotation
        self.publisher_cmd_vel.publish(twist)
        self.get_logger().info("Stopping rotation.")
            
        # Recovery completed, allow normal navigation again
        self.is_in_recovery = False
        self.recovery_completed = True  # Mark recovery as completed     
        
        
    
def main(args=None):
    rclpy.init(args=args)
    navigator = ProjectedMapNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
