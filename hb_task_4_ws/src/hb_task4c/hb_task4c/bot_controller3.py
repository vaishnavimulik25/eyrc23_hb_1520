
#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ hb_1520 ]
# Author List:		[ Jiten,Vaishnavi ]
# Filename:		feedback.py
# Functions:
# [ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

from cv_bridge import CvBridge
import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import time
import math
from my_robot_interfaces.msg import Goal
import numpy as np
from collections import deque


class HBController1(Node):
    def __init__(self):
        super().__init__('hb_controller1')

        # Initialze Publisher and Subscriber
        self.subscription = self.create_subscription(
            Goal,
            'hb_bot_3/goal',
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )
        self.aruco_subscriber = self.create_subscription(
            Pose2D, '/pen3_bot', self.aruco_callback, 10)
        self.vel_rear_msg = self.create_publisher(
            Twist, '/cmd_vel/bot3', 10)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.linear_thresh = 5
        self.ang_thresh = 0.1
        # Initialize a Twist message for velocity commands
        self.rear_pub = Twist()

        # Initialize required variables
        self.x_error = 0.0
        self.y_error = 0.0
        self.theta_error = 0.0

        self.a = 0
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.k_linear = 0.08
        self.hb_theta = 0.0

        self.k_angular = -0.01
        self.v_left = 0.0
        self.v_right = 0.0
        self.v_rear = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_theta = 0.0
        self.bot_1_x = [340.0]
        self.bot_1_y = [366.0]
        self.bot_1_theta = [0.0]

    def aruco_callback(self, msg):
        # Update the position and orientation fromaruco message
        self.hb_x = msg.x  # origin shifted
        self.hb_y = msg.y
        self.hb_theta = msg.theta
        # self.get_logger().info('Aruco call')

    def distance(self, x, y):
        return abs(math.sqrt((self.hb_x - x)**2 + (self.hb_y - y)**2))

    def calculate_velocity_commands(self, x, y, th):
        # Calculate Error from feedback
        self.x_error = (x - self.hb_x)
        self.y_error = (y - self.hb_y)
        self.theta_error = (th - self.hb_theta)

        # Change the frame by using Rotation Matrix (If you find it required)
        self.robot_frame_x_vel = (self.x_error * math.cos(self.hb_theta)) + \
            (self.y_error * math.sin(self.hb_theta))
        self.robot_frame_y_vel = (self.y_error * math.cos(self.hb_theta)) - \
            (self.x_error * math.sin(self.hb_theta))

        # Calculate the required velocity of bot for the next iteration(s)
        self.vel_x = self.k_linear * self.robot_frame_x_vel
        self.vel_y = self.k_linear * self.robot_frame_y_vel
        self.vel_theta = self.k_angular * self.theta_error

        self.rear_pub.linear.x = self.vel_x
        self.rear_pub.linear.y = self.vel_y
        self.rear_pub.angular.z = self.vel_theta

        self.cmd_vel_pub.publish(self.rear_pub)

#        # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
#        self.inverse_kinematics()
#
#        # Apply appropriate force vectors
#        self.vel_left_msg.force.y = self.v_left
#        self.vel_right_msg.force.y = self.v_right
#        self.vel_rear_msg.force.y = self.v_rear
#
#        # Publish the calculated efforts to actuate robot by applying force vectors on provided topics
#        self.left_pub.publish(self.vel_left_msg)
#        self.right_pub.publish(self.vel_right_msg)
#        self.rear_pub.publish(self.vel_rear_msg)
#        # self.get_logger().info('velocity published')

    def inverse_kinematics(self):
        # Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        r = 0.14
        l = 0.68
        self.matrix_3x3 = np.negative(np.array([[1 / r, -math.sqrt(3) / r, -l / r],
                                                [1 / r,
                                                    math.sqrt(3) / r, -l / r],
                                                [-2 / r,  0.0, -l / r]]))  # seq right, left, rear
        self.vector_3x1 = np.array([self.vel_x, self.vel_y, self.vel_theta])
        self.result = np.dot(self.matrix_3x3, self.vector_3x1)
        self.v_left, self.v_right, self.v_rear = self.result

    def goalCallBack(self, msg):
        self.bot_1_x.extend(msg.x)
        self.bot_1_y.extend(msg.y)
        self.bot_1_theta.append(msg.theta)
        self.get_logger().info('goal callback')
        self.a = self.a+1


def main(args=None):
    rclpy.init(args=args)
    hb_controller1 = HBController1()
    second = 0
    third = time.time()
    # Main loop
    while rclpy.ok():
        first = time.time()
        if len(hb_controller1.bot_1_x) != 0 and len(hb_controller1.bot_1_y) != 0 and len(hb_controller1.bot_1_theta) != 0 and (first - third) > 5:
            hb_controller1.calculate_velocity_commands(hb_controller1.bot_1_x[0], hb_controller1.bot_1_y[0], hb_controller1.bot_1_theta[0])

        if abs(hb_controller1.x_error) <= hb_controller1.linear_thresh and abs(hb_controller1.y_error) <= hb_controller1.linear_thresh and hb_controller1.a != 0 and (first - second) > 1 and len(hb_controller1.bot_1_x) != 0 and len(hb_controller1.bot_1_y) != 0 and len(hb_controller1.bot_1_theta) != 0:
            second = time.time()
            # print(hb_controller1.bot_1_x)
            hb_controller1.bot_1_x = deque(hb_controller1.bot_1_x)
            hb_controller1.bot_1_y = deque(hb_controller1.bot_1_y)
            hb_controller1.bot_1_theta = deque(hb_controller1.bot_1_theta)
            hb_controller1.bot_1_x.popleft()
            hb_controller1.bot_1_y.popleft()
            hb_controller1.bot_1_theta.popleft()
            hb_controller1.get_logger().info("Goal reached")

        rclpy.spin_once(hb_controller1)
    # Destroy the node and shut down ROS
    hb_controller1.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == '__main__':
    main()
