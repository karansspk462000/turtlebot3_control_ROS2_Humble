#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
from threading import Thread, Event
import signal
import sys

class Turtle(Node):
    """
    A ROS 2 node that controls a robot's movement towards a specified goal position.
    
    Attributes:
        pub: Publisher object to publish Twist messages to the '/cmd_vel' topic.
        sub: Subscriber object to subscribe to Odometry messages from the '/odom' topic.
        current_pose: The current pose of the robot as received from the Odometry message.
        goal_pose: The target pose that the robot should move towards.
        goal_set: A flag indicating if the goal position has been set.
        input_thread: Thread for handling user input to set the goal position.
        stop_event: Event object to signal the thread to stop.
    """
    
    def __init__(self):
        """
        Initialize the Turtle node, set up publishers, subscribers, and start the input thread.
        """
        super().__init__('turtle')

        # Quality of Service profile for reliable communication
        qos_profile = QoSProfile(depth=10)

        # Create a publisher to send velocity commands to the '/cmd_vel' topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Create a subscription to receive Odometry messages from the '/odom' topic
        self.sub = self.create_subscription(Odometry, '/odom', self.pose_callback, qos_profile)

        # Initialize current pose and goal pose
        self.current_pose = Odometry()
        self.goal_pose = Odometry()
        self.goal_set = False

        # Create an Event object to signal the input thread to stop
        self.stop_event = Event()

        # Start a separate thread for handling user input
        self.input_thread = Thread(target=self.set_goal_pose)
        self.input_thread.start()

        # Register signal handler for graceful shutdown on Ctrl+C
        # signal.signal(signal.SIGINT, self.signal_handler)

    def set_goal_pose(self):
        """
        Continuously prompts the user to enter goal coordinates and updates the goal pose.
        This method runs in a separate thread and checks for stop signals.
        """
        while not self.stop_event.is_set():
            try:
                # Prompt user for goal coordinates
                self.goal_pose.pose.pose.position.x = float(input("Enter x coordinate of goal: "))
                self.goal_pose.pose.pose.position.y = float(input("Enter y coordinate of goal: "))
                self.goal_set = True
            except ValueError:
                # Log a warning if the input is not numeric
                self.get_logger().warn('Invalid input. Please enter numeric values.')

    def update_pose(self, data):
        """
        Update the current pose with data from Odometry messages.
        
        Args:
            data (Odometry): The Odometry message containing the current pose.
        """
        # Round position coordinates to 4 decimal places
        data.pose.pose.position.x = round(data.pose.pose.position.x, 4)
        data.pose.pose.position.y = round(data.pose.pose.position.y, 4)
        self.current_pose = data

    def pose_callback(self, data):
        """
        Callback function for the Odometry subscriber. Updates the current pose.
        
        Args:
            data (Odometry): The Odometry message containing the new pose.
        """
        self.update_pose(data)

    def calculate_distance(self):
        """
        Calculate the distance between the current pose and the goal pose.
        
        Returns:
            float: The Euclidean distance to the goal.
        """
        return math.sqrt(
            (self.goal_pose.pose.pose.position.x - self.current_pose.pose.pose.position.x) ** 2 +
            (self.goal_pose.pose.pose.position.y - self.current_pose.pose.pose.position.y) ** 2
        )

    def calculate_error_angle(self):
        """
        Calculate the angular error between the robot's current orientation and the direction to the goal.
        
        Returns:
            float: The angular error in radians.
        """
        desired_angle = math.atan2(
            self.goal_pose.pose.pose.position.y - self.current_pose.pose.pose.position.y,
            self.goal_pose.pose.pose.position.x - self.current_pose.pose.pose.position.x
        )

        # Quaternion to rotation matrix conversion
        q0 = self.current_pose.pose.pose.orientation.w
        q1 = self.current_pose.pose.pose.orientation.x
        q2 = self.current_pose.pose.pose.orientation.y
        q3 = self.current_pose.pose.pose.orientation.z

        R_des = np.array([[math.cos(desired_angle), -math.sin(desired_angle), 0],
                          [math.sin(desired_angle),  math.cos(desired_angle), 0],
                          [0, 0, 1]])

        R_curr = np.array([
            [1 - 2*q2*q2 - 2*q3*q3, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
            [2*q1*q2 + 2*q0*q3, 1 - 2*q1*q1 - 2*q3*q3, 2*q2*q3 - 2*q0*q1],
            [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1*q1 - 2*q2*q2]
        ])

        R_curr_T = np.transpose(R_curr)
        R_error = np.dot(R_des, R_curr_T)
        error_angle = math.atan2(R_error[1][0], R_error[0][0])
        
        return error_angle

    def control_loop(self):
        """
        Main control loop that sends velocity commands to move the robot towards the goal.
        """
        msg = Twist()

        while rclpy.ok() and not self.stop_event.is_set():
            if not self.goal_set:
                # If the goal is not set, continue spinning to process incoming messages
                rclpy.spin_once(self)
                continue

            dist = self.calculate_distance()

            if dist > 1e-3:
                # Set linear velocity proportional to the distance
                msg.linear.x = 0.2 * dist
                # Set angular velocity proportional to the error angle
                msg.angular.z = 0.6 * self.calculate_error_angle()
                
                # Log the status
                self.get_logger().info(f"Distance from goal: {dist}")
                # self.get_logger().info(f"Linear velocity: {msg.linear.x}")
                # self.get_logger().info(f"Angular velocity: {msg.angular.z}")

                # If angular error is within a small threshold, stop rotation
                if -4 < math.degrees(msg.angular.z) < 4:
                    msg.angular.z = 0.0

                self.pub.publish(msg)
            else:
                # Stop all motion if the goal is reached
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.pub.publish(msg)
                self.get_logger().info("Reached the goal")
                break

            # Spin once with a timeout to avoid blocking the control loop
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Control loop ended")
        

    def signal_handler(self, sig, frame):
        """
        Handle interrupt signal (Ctrl+C) to ensure graceful shutdown.
        
        Args:
            sig (int): The signal number.
            frame (signal frame): The current stack frame.
        """
        self.get_logger().info("Interrupt received. Shutting down gracefully...")
        self.stop_event.set()  # Signal the input thread to stop
        self.input_thread.join()  # Wait for the input thread to finish
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    """
    Main entry point for the ROS 2 node. Initializes and runs the Turtle node.
    """
    rclpy.init(args=args)
    turtle = Turtle()

    try:
        # Run the control loop
        turtle.control_loop()
    except KeyboardInterrupt:
        # Handle keyboard interrupt gracefully
        turtle.get_logger().info("KeyboardInterrupt received. Shutting down gracefully...")
        turtle.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()
