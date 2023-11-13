#! /usr/bin/env python3

'''
*******************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*******************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
# [ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
import numpy as np

# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')

        # Initialze Publisher and Subscriber
        self.aruco_subscriber = self.create_subscription(
            Pose2D, "/detected_aruco", self.aruco_callback, 10)
        self.left_pub = self.create_publisher(
            Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.right_pub = self.create_publisher(
            Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.rear_pub = self.create_publisher(
            Wrench, '/hb_bot_1/rear_wheel_force', 10)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        # Initialize a Twist message for velocity commands
        self.vel_left_msg = Wrench()
        self.vel_right_msg = Wrench()
        self.vel_rear_msg = Wrench()

        # Initialize required variables
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0
        self.k_linear = 0.1
        self.k_angular = -15.0
        self.v_left = 0.0
        self.v_right = 0.0
        self.v_rear = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_theta = 0.0

        # client for the "next_goal" service ie provide type and name of the
        # service ,cli is object representing the client
        self.cli = self.create_client(NextGoal, 'next_goal')
        # create a request
        self.req = NextGoal.Request()
        self.index = 0

    def aruco_callback(self, msg):
        # Update the position and orientation fromaruco message
        self.hb_x = msg.x  # origin shifted
        self.hb_y = msg.y
        self.hb_theta = msg.theta

    def distance(self, x, y):
        return abs(math.sqrt((self.hb_x - x)**2 + (self.hb_y - y)**2))

    def calculate_velocity_commands(self, x, y, th):
        # Calculate Error from feedback
        x_error = (x - self.hb_x)
        y_error = (y - self.hb_y)
        theta_error = (th - self.hb_theta)

        # Change the frame by using Rotation Matrix (If you find it required)
        robot_frame_x_vel = (x_error * math.cos(self.hb_theta)) - \
            (y_error * math.sin(self.hb_theta))
        robot_frame_y_vel = (y_error * math.cos(self.hb_theta)) - \
            (x_error * math.sin(self.hb_theta))

        # Calculate the required velocity of bot for the next iteration(s)
        self.vel_x = self.k_linear * robot_frame_x_vel
        self.vel_y = self.k_linear * robot_frame_y_vel
        self.vel_theta = self.k_angular * theta_error

        # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
        self.inverse_kinematics()

        # Apply appropriate force vectors
        self.vel_left_msg.force.y = self.v_left
        self.vel_right_msg.force.y = self.v_right
        self.vel_rear_msg.force.y = self.v_rear

        # Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        self.left_pub.publish(self.vel_left_msg)
        self.right_pub.publish(self.vel_right_msg)
        self.rear_pub.publish(self.vel_rear_msg)

    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        # fill the request which was made earlier
        self.req.request_goal = request_goal
        # calls the service in asynchronous way,future calls callback which can
        # can be used to change the settings
        self.future = self.cli.call_async(self.req)

    def inverse_kinematics(self):
        # Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        # matrix_3x3 = -np.array([[7.145919679862797, -12.376237623762377,-4.8615170766646765],[7.145919679862797, 12.376237623762377,-4.8615170766646765],[-14.291839, 0.0, -4.861517076]])
        r = 0.14
        l = 0.68
        matrix_3x3 = np.negative(np.array([[1 / r, -math.sqrt(3) / r, -l / r],
                                           [1 / r,  math.sqrt(3) / r, -l / r],
                                           [-2 / r,  0.0, -l / r]]))  # seq right, left, rear
        # matrix_3x3 = (np.array([[1 / r, math.sqrt(3) / r, -l / r],
        #                                    [1 / r,  -math.sqrt(3) / r, -l / r],
        #                                    [-2 / r,  0.0, -l / r]]))
        vector_3x1 = np.array([self.vel_x, self.vel_y, self.vel_theta])
        result = np.dot(matrix_3x3, vector_3x1)
        # self.v_right, self.v_left, self.v_rear = result
        self.v_left, self.v_right, self.v_rear = result


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the HBController class
    hb_controller = HBController()

    # Main loop
    while rclpy.ok():

        if hb_controller.cli.wait_for_service(timeout_sec=1.0):
            hb_controller.send_request(hb_controller.index)
            rclpy.spin_until_future_complete(
                hb_controller, hb_controller.future)
        # Check if the service call is done
            if hb_controller.future.done():
                try:
                    # response from the service call of form x,y,theta goal
                    response = hb_controller.future.result()
                except Exception as e:
                    hb_controller.get_logger().infselfo(
                        'Service call failed %r' % (e,))
                else:
                    #########           GOAL POSE             #########
                    x_goal = response.x_goal + 250
                    y_goal = response.y_goal + 250
                    theta_goal = 0
                    hb_controller.flag = response.end_of_list
                ####################################################

                    hb_controller.calculate_velocity_commands(
                        x_goal, y_goal, theta_goal)

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)

                ############     DO NOT MODIFY THIS       #########
                if hb_controller.distance(x_goal, y_goal) < 5:
                    hb_controller.index += 1
                    if hb_controller.flag == 1:
                        hb_controller.index = 0
                    hb_controller.get_logger().info("goal reached")
                ####################################################

                hb_controller.get_logger().info("done")
        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
        hb_controller.rate.sleep()

    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()


# Entry point of the script
if __name__ == '__main__':
    main()
