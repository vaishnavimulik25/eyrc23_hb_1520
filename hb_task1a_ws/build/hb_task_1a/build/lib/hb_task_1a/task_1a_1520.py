#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import math
from turtlesim.srv import Spawn, Kill
#1
def main():


    rclpy.init()
    node = rclpy.create_node('circle_motion_node')
    
    

    # Create a publisher to send Twist commands to the turtle
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Create a Twist message for moving in a circle
    twist1 = Twist()
    twist1.linear.x = 2.0  # Linear velocity (forward speed)
    twist1.angular.z = 2.0  # Angular velocity (rotation rate)

    # Set the time to move in a circle (in seconds) and calculate the number of revolutions
    time_to_move = 10.0  # Adjust this value as needed
    rate = node.create_rate(10)
    num_revolutions = time_to_move * twist1.angular.z / (2 * math.pi)

    # Publish the Twist message to make the turtle move in a circle
    revolutions_completed = 0.0
    while rclpy.ok() and revolutions_completed < num_revolutions:
        publisher.publish(twist1)
        node.get_logger().info('Moving in a circle...')
        rclpy.spin_once(node)
        revolutions_completed += 0.1  # Assuming a control rate of 10 Hz

    # Stop the turtle when it completes one full circle
    twist1.linear.x = 0.0
    twist1.angular.z = 0.0
    publisher.publish(twist1)








    spawn_client = node.create_client(Spawn, '/spawn')
    while not spawn_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn service...')
    spawn_request = Spawn.Request()
    spawn_request.x = 5.44445  # X-coordinate for the new turtle's spawn location
    spawn_request.y = 5.44445  # Y-coordinate for the new turtle's spawn location
    spawn_request.name = 'turtle2'
    spawn_request.theta = 0.0  # Initial orientation (0 radians)
    spawn_future = spawn_client.call_async(spawn_request)
    
    
    publisher = node.create_publisher(Twist, '/turtle2/cmd_vel', 10)

    # Create a Twist message for moving in a circle
    twist2 = Twist()
    twist2.linear.x = 2.0  # Linear velocity (forward speed)
    twist2.angular.z = -2.0  # Angular velocity (rotation rate)

    #Set the time to move in a circle (in seconds) and calculate the number of revolutions
    time_to_move = 10.0  # Adjust this value as needed
    rate = node.create_rate(10)
    num_revolutions = time_to_move * twist2.angular.z / (2 * math.pi)
   

    # Publish the Twist message to make the turtle move in a circle
    revolutions_completed = math.pi
    while rclpy.ok() and revolutions_completed > num_revolutions:
        publisher.publish(twist2)
        node.get_logger().info('Moving in a circle...')
        rclpy.spin_once(node)
        revolutions_completed -= 0.1  # Assuming a control rate of 10 Hz
        
        
           
    #Stop the turtle when it completes one full circle
    twist2.linear.x = 0.0
    twist2.angular.z = 0.0
    publisher.publish(twist2)


    # Wait for the spawn request to complete
    rclpy.spin_until_future_complete(node, spawn_future)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
