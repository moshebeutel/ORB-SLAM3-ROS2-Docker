import threading
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from slam_msgs.msg import MapData, MapGraph
import rclpy
from rclpy.node import Node

class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.cmd_pub = self.create_publisher(Twist, '/robot_0/cmd_vel', 10)
        self.robot_position = np.array([0, 0, 0])  # Placeholder for robot's current position
        self.obstacle_threshold = 0.25  # meters
        self.turn_angle = 30  # Degrees for turn
        
        self.map_data_subscriber = self.create_subscription(
            MapData,
            '/robot_0/map_data',  # Assuming this is the correct topic for map data
            self.map_data_callback,
            10
        )
        
        self.robot_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/robot_0/robot_pose_slam',  # Assuming this is the correct topic for map data
            self.robot_pose_callback,
            10
        )

        self.twist = Twist()
        self.poses_list = []  # This will hold the poses from ORB-SLAM3
        self.lock = threading.Lock()  # Lock to synchronize access to the list
        self.robot_pose_lock = threading.Lock()

    def map_data_callback(self, msg: MapData):
        # Store the map data for processing
        with self.lock:  # Acquire the lock before modifying the list
            # Add the new poses to the list (or process them)
            self.poses_list = msg.graph.poses
            self.get_logger().info(f"Received {len(msg.graph.poses)} poses")

    def robot_pose_callback(self, msg: PoseStamped):
        # Store the map data for processing
        with self.robot_pose_lock:  # Acquire the lock before modifying the list
            # Update robot Pose
            self.robot_position = np.array([msg.pose.position.x, 
                                            msg.pose.position.y, 
                                            msg.pose.position.z])
        
   
    def get_poses(self):
        with self.lock:  # Acquire the lock before reading the list
            return self.poses_list.copy()  # Return a copy to avoid direct modification
        

    def check_for_obstacles(self):
        """
        Check for obstacles within an ROI. If a free direction is found, move forward.
        """
        poses_list = self.get_poses()
        roi_radius = 1.0  # Adjust based on the environment (meters)
        cur_pose = self.robot_position
        # Start checking from the last pose in reverse order
        for pose in poses_list[-10:]:
            cur_pose = pose.pose
            cur_pose_position = np.array([cur_pose.position.x, cur_pose.position.y, cur_pose.position.z])

            # Check if the pose is within the ROI
            if np.linalg.norm(self.robot_position - cur_pose_position) > roi_radius:
                continue  # Ignore points outside the region of interest

            # Compute Euclidean distance to the pose
            distance = np.linalg.norm(self.robot_position - cur_pose_position)
            self.get_logger().info(f"Distance is {distance}")

            if distance > self.obstacle_threshold:
                return False, cur_pose  # Found a free direction

        return True, None  # No free direction found, an obstacle is blocking

    
    def decide_movement(self):
        """
        Decide whether to move forward or turn based on proximity of poses.
        """
        obstacle_detected, closest_pose = self.check_for_obstacles()
        
        if obstacle_detected:
            # If the closest pose is ahead, turn right or left
            angle_to_turn = self.turn_angle  # Placeholder, could be dynamic based on pose position
            self.turn_robot(angle_to_turn)
        else:
            # No obstacle, move forward
            self.move_forward()

    def move_forward(self):
        """
        Command the robot to move forward.
        """
        self.publish_velocity(linear=0.2)
        self.get_logger().info("Moving forward")

    def turn_robot(self, angle):
        """
        Command the robot to turn.
        """
        self.publish_velocity(angular=float(0.3))
        self.get_logger().info(f"Turning {angle} degrees")
    
    def publish_velocity(self, linear=0.0, angular=0.0):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.cmd_pub.publish(self.twist)
        # self.get_logger().info(f"Publishing velocity: Linear: {linear}, Angular: {angular}")
        

    def run(self):
        """
        Main loop to decide movement.
        """
        while rclpy.ok():
            self.decide_movement()
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    controller = MovementController()
    controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

