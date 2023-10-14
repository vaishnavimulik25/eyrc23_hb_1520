#!/usr/bin/env python3
import rclpy                                         # ROS 2 Python library for creating ROS 2 nodes
from rclpy.node import Node                          # Node class for creating ROS 2 nodes
from geometry_msgs.msg import Twist                   # Publishing to /cmd_vel with msg type: Twist
from nav_msgs.msg import Odometry                    # Subscribing to /odom with msg type: Odometry
import time                                          # Python time module for time-related functions
import math                                          # Python math module for mathematical functions
from tf_transformations import euler_from_quaternion # Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from my_robot_interfaces.srv import NextGoal          # Service



class HBTask1BController(Node):
    def __init__(self):
        super().__init__('hb_task1b_controller')
        self.get_logger().info("Controller node has been started")
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10)
        
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        # x_goal=0
        # y_goal=0
        # theta_goal=0

        # Initialize required variables
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0
        self.k_linear = 1
        self.k_angular = 1

        # Initialize a Twist message for velocity commands
        self.vel_msg = Twist()

        # Create a client for the "next_goal" service
        self.client = self.create_client(NextGoal, 'next_goal')
        self.index = 0

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def odometry_callback(self, msg):
        # Update the position and orientation from the Odometry message
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y
        self.hb_theta = msg.twist.twist.angular.z
        
        # Implement your control logic here
        # self.calculate_velocity_commands (x_goal,y_goal,theta_goal)

    def distance(self):
        return abs(math.sqrt((self.hb_x - self.x_goal) ** 2 + (self.hb_y - self.y_goal) ** 2))

    def calculate_velocity_commands(self, x, y, th):
        # Implement your control logic to calculate vel_msg based on the current position and goal
        # if self.hb_x < 3.0:
        self.vel_msg.linear.x = self.k_linear * (x - self.hb_x)
        self.vel_msg.linear.y = self.k_linear * (y - self.hb_y)
        self.vel_msg.angular.z = self.k_angular * (th - self.hb_theta)
        # else:
        #     self.vel_msg.linear.y = -3.0
        #     self.vel_msg.angular.z = 0.001
        self.cmd_vel_pub.publish(self.vel_msg)
        self.get_logger().info('vel published')

# Note: You'll need to integrate this code with your robot's sensors and odometry to obtain the current position and
# continuously update the velocity commands as the robot moves.
    def send_request(self, index):
        # Send a request to the "next_goal" service
        request = NextGoal.Request()
        request.request_goal = index
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()

    while rclpy.ok():
        if ebot_controller.client.wait_for_service(timeout_sec=1.0):
            future = ebot_controller.send_request(ebot_controller.index)
            rclpy.spin_until_future_complete(ebot_controller, future)

            if future.done():
                try:
                    response = future.result()
                    # Extract goal pose information from the response
                except Exception as e:
                    ebot_controller.get_logger().info('Service call failed %r' % (e,))
                    
                else:
                    x_goal = response.x_goal
                    y_goal = response.y_goal
                    theta_goal = response.theta_goal
                    flag = response.end_of_list
                    
                    # Implement your control logic here
                    ebot_controller.calculate_velocity_commands(x_goal,y_goal,theta_goal)
                    ebot_controller.index += 1
                    if flag == 1 :
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)
        # time.sleep(1)
        # rclpy.spin_once(ebot_controller)
        # ebot_controller.rate.sleep()


    # rclpy.spin(ebot_controller)
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
