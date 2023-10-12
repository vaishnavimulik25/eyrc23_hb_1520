#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):
    def __init__(self):
        super().__init__('hb_task1b_controller')
        self.get_logger().info("Controller node has been started")
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, 10)

        # Initialize required variables
        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0

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
        self.calculate_velocity_commands()

    def calculate_velocity_commands(self):
        # Implement your control logic to calculate vel_msg based on the current position and goal

        # Example: Set linear velocity to 0.2 m/s
        self.vel_msg.linear.x = 0.5

        # Example: Set angular velocity to 0.1 rad/s
        self.vel_msg.angular.z = 0.001

        # Publish the velocity commands
        self.cmd_vel_pub.publish(self.vel_msg)

    def send_request(self):
        # Send a request to the "next_goal" service
        request = NextGoal.Request()
        request.request_goal = self.index

        future = self.client.call_async(request)

        return future

def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()

    while rclpy.ok():
        if ebot_controller.client.wait_for_service(timeout_sec=1.0):
            future = ebot_controller.send_request()
            rclpy.spin_until_future_complete(ebot_controller, future)

            if future.done():
                try:
                    response = future.result()

                    # Extract goal pose information from the response
                    x_goal = response.x_goal
                    y_goal = response.y_goal
                    theta_goal = response.theta_goal

                    # Implement your control logic here
                    ebot_controller.calculate_velocity_commands()

                except Exception as e:
                    ebot_controller.get_logger().info('Service call failed %r' % (e,))

    rclpy.spin(ebot_controller)
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
