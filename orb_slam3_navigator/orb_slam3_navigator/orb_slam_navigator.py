import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from slam_msgs.msg import MapData, MapGraph, KeyFrame
from geometry_msgs.msg import PoseStamped, Twist
from time import time


class OrbSLAMNavigator(Node):
    def __init__(self):
        super().__init__('orb_slam_navigator')
        self.turn_left = True
        self.turn_counter = 0
        self.max_turns = 12  # For 360 degrees, with 30 degrees per turn
        self.obstacle_threshold = 35.0  # Distance threshold to check for obstacles (in cm) 
        self.last_obstacle_time = time()
       
       # Timer to periodically check and act
        self.create_timer(0.1, self.main_loop)  # 0.1s loop to check for obstacles and move
        self.map_data_subscriber = self.create_subscription(
            MapData,
            '/map_data',  # Assuming this is the correct topic for map data
            self.map_data_callback,
            10
        )

        self.map_graph = None
        self.keyframes = None
        self.twist = Twist()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def map_data_callback(self, msg: MapData):
        # Store the map data for processing
        self.map_graph = msg.graph
        self.keyframes = msg.nodes

    def turn(self):
        # Check if we have map data
        if not self.map_graph and not self.keyframes:
            self.get_logger().warn("No map data available. Cannot navigate.")
            return

        # Check if there is an obstacle in front using map data (this is simplified for now)
        if self.is_obstacle_in_front():
            # Turn in the same direction (left or right)
            if self.turn_left:
                self.publish_velocity(0.0, 0.3)  # Turn left
            else:
                self.publish_velocity(0.0, -0.3)  # Turn right

            self.turn_counter += 1
            self.get_logger().info(f"Turning... (turn #{self.turn_counter})")

            # If the turn counter reaches the max, stop turning or perform another action
            if self.turn_counter >= self.max_turns:
                self.turn_counter = 0  # Reset the counter after a full 360 degrees turn
                self.get_logger().info("Completed 360 degrees of turning.")

                # Check if there is still an obstacle after turning
                if self.is_obstacle_in_front():
                    self.get_logger().warn("Warning: Still an obstacle in front after turning 360 degrees.")
                    # Optionally: Ask for help, or call a reset behavior, etc.
                    self.stop_turning()
                    self.request_help()  # This could be a call to a reset service or human assistance
                else:
                    self.get_logger().info("Clear path ahead. Proceeding with forward movement.")
                    self.move_forward()
        else:
            self.get_logger().info("No obstacle detected. Proceeding forward.")
            self.move_forward()

    def stop_turning(self):
        self.publish_velocity(0.0, 0.0)  # Stop turning
        self.get_logger().info("Stopped turning.")

    def is_obstacle_in_front(self):
        # Use the map data (or a sensor fusion approach) to determine if there's an obstacle in front.
        # This is a simplified example that can be expanded based on actual map data and navigation algorithms.
        if not self.keyframes:
            return False  # No keyframes to check, assume no obstacle.

        # Assuming keyframes or poses contain obstacle information (or proximity data)
        for keyframe in self.keyframes:
            # Check the poses to see if any point falls within the robot's range (e.g., 35cm)
            for point in keyframe.word_pts:
                distance = self.calculate_distance_to_point(point)
                if distance < self.obstacle_threshold:
                    self.get_logger().info(f"Obstacle detected at {distance} cm.")
                    return True
        return False

    def request_help(self):
        # This is a placeholder for any help-request logic you might want to implement,
        # like sending a message, calling a service, or triggering a reset behavior.
        self.get_logger().warn("Requesting help: Robot is stuck!")

    def calculate_distance_to_point(self, point):
        # This is a simplified example of distance calculation from the robot's current position
        # Assuming the robot is at the origin (0,0) for simplicity
        return (point.x**2 + point.y**2)**0.5

    def move_forward(self):
        self.publish_velocity(0.5, 0.0)  # Move forward with a speed of 0.5
        self.get_logger().info("Moving forward.")

    def publish_velocity(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.cmd_pub.publish(self.twist)
        self.get_logger().info(f"Publishing velocity: Linear: {linear}, Angular: {angular}")
    
    def main_loop(self):
        """Main loop to check obstacles and take actions"""
        if self.check_obstacles():
            self.turn()  # Turn if there's an obstacle or we're stuck
        else:
            self.publish_velocity(0.2, 0.0)  # Move forward if no obstacle detected

    def check_obstacles(self):
        """Check if the robot is stuck or facing an obstacle"""
        # In this case, we're just using the obstacle distance threshold
        # You could use the map data to check if an obstacle is near
        if self.is_obstacle_in_front():
            self.last_obstacle_time = time()  # Reset the timer since we detected an obstacle
            return True
        elif time() - self.last_obstacle_time > 3:  # 3 seconds of no movement means stuck
            self.get_logger().warn("Robot seems stuck! Initiating turn.")
            return True
        return False



def main(args=None):
    rclpy.init(args=args)
    navigator = OrbSLAMNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
