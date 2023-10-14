#!/usr/bin/env python3

import rclpy                                         # ROS 2 Python library for creating ROS 2 nodes
from rclpy.node import Node                          # Node class for creating ROS 2 nodes
from geometry_msgs.msg import Twist                   # Publishing to /cmd_vel with msg type: Twist
from nav_msgs.msg import Odometry                    # Subscribing to /odom with msg type: Odometry
import time                                          # Python time module for time-related functions
import math                                          # Python math module for mathematical functions
from tf_transformations import euler_from_quaternion # Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from my_robot_interfaces.srv import NextGoal          # Service
from functools import partial


class HBTask1BController(Node):
    def __init__(self):
        super().__init__('hb_task1b_controller')
        self.get_logger().info("Controller node has been started")
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometryCb, 10)
        self.index = 0
        # Declare a Twist message
        self.vel_msg = Twist()
        
        # Initialise the required variables to 0
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0
        
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        
        # Initializing variables used for control loop (x_d, y_d, theta_d)
        self.x_goal = [0, -1, -1, 1, 0]
        self.y_goal = [2, 1, -1, -1, 0]
        self.theta_goal = [0, math.pi/2, -math.pi, -math.pi/2, 0]
        self.Kp_linear = 1.0
        self.Kp_angular = 1.0
        
        self.x_d = self.x_goal[self.index]
        self.y_d = self.y_goal[self.index]
        self.theta_d = self.theta_goal[self.index]

        # initialising publisher and subscriber of cmd_vel and odom respectively

        
        # Create a client for the "next_goal" service
        self.client = self.create_client(NextGoal, 'next_goal')
        self.req = NextGoal.Request() 
        # self.index = 0
        self.future = None  # Initialize the future variable

    def odometryCb(self, msg):
        # global hb_x, hb_y, hb_theta, x_d, y_d, theta_d
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Update the position and orientation from the Odometry message
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y
        self.hb_theta = yaw
        # Implement your control logic here


    def calculate_velocity_commands(self):

        # Calculate errors in global frame
        error_x = self.x_d - self.hb_x
        error_y = self.y_d - self.hb_y
        error_theta = self.theta_d - self.hb_theta
        # Calculate velocity commands using a P controller
        self.vel_msg.linear.x = self.Kp_linear * error_x
        self.vel_msg.linear.y = self.Kp_linear * error_y
        self.vel_msg.angular.z = self.Kp_angular * error_theta
        # Publish the velocity commands
        self.cmd_vel_pub.publish(self.vel_msg)
        
    def is_goal_reached(self):
        distance = abs(((self.hb_x - self.x_d) ** 2 + (self.hb_y - self.y_d) ** 2) ** (0.5))
        return distance < 0.1

    def send_request(self, x, y, th):
        self.req.request_goal = self.index
        self.future = self.client.call_async(self.req)
        self.x_d = x[self.req.request_goal]
        self.y_d = y[self.req.request_goal]
        self.theta_d = th[self.req.request_goal]

        # self.client.call_async(self.req, callback=partial(self.next_goal_callback))
        # self.future.add_done_callback(self.handle_response)
    
    # def handle_response(self, future):
    #     try:
    #         response=future.result()
    #     except Exception as e:
    #         self.get_logger().info('Service call failed %r' % (e,))
        
            
def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()
    ebot_controller.send_request(ebot_controller.x_goal,ebot_controller.y_goal,ebot_controller.theta_goal)

    while rclpy.ok():
        if ebot_controller.client.wait_for_service(timeout_sec=1.0):
            if ebot_controller.future is not None and ebot_controller.future.done():
                try:
                    response = ebot_controller.future.add_done_callback(ebot_controller.req, response)
                    ebot_controller.get_logger().info("Received response")
                    # Extract goal pose information from the response
                    # Implement your control logic here
                    # ebot_controller.calculate_velocity_commands()
                except Exception as e:
                    ebot_controller.get_logger().info('Service call failed %r' % (e,))
                    ebot_controller.future = None

                else:
                    ebot_controller.get_logger().error("index is", ebot_controller.index)
                    # ebot_controller.future.add_done_callback(ebot_controller.req, response)
                    # ebot_controller.index = response.next_goal
                    # ebot_controller.x_goal[ebot_controller.index] = response.x_goal
                    # ebot_controller.y_goal[ebot_controller.index] = response.y_goal
                    # ebot_controller.theta_goal = response.theta_goal
                    # ebot_controller.flag = response.end_of_list
                    # ebot_controller.calculate_velocity_commands()
                    #If Condition is up to you
                    ############     DO NOT MODIFY THIS       #########
                    # ebot_controller.send_request(ebot_controller.x_goal,ebot_controller.y_goal,ebot_controller.theta_goal)
                    ####################################################
                if ebot_controller.is_goal_reached():
                    ebot_controller.index += 1
                    if ebot_controller.index >= len(ebot_controller.x_goal) :
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.x_goal,ebot_controller.y_goal,ebot_controller.theta_goal)
            ebot_controller.calculate_velocity_commands()
            # self.send_request()
            rclpy.spin_once(ebot_controller)
            ebot_controller.rate.sleep()
        
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

