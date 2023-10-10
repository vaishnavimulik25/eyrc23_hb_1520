
#!/usr/bin/env python3

'''
########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID:
# Team Leader Name:
# Team Members Name:
# College:
########################################################################################################################

'''


import rclpy
from geometry_msgs.msg import Twist
import math
from turtlesim.srv import Spawn, Kill

def main():
    rclpy.init()

    node = rclpy.create_node('circle_motion_node')
    
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)         # Create a publisher to send Twist commands to the turtle
    
    twist1 = Twist()                                                         # Create a Twist message for moving in a circle
    twist1.linear.x = 2.0                                                    # Linear velocity (forward speed)
    twist1.angular.z = 2.0                                                   # Angular velocity (rotation rate)
    
    time_to_move = 10.0                                                      # Set the time to move in a circle (in seconds) and calculate the number of revolutions
    rate = node.create_rate(10)
    num_revolutions = time_to_move * twist1.angular.z / (2 * math.pi)
    



    revolutions_completed = 0.0                                             # Setting the intial rovolution value to 0 
    while rclpy.ok() and revolutions_completed < num_revolutions:
        publisher.publish(twist1)                                           # Publish the Twist message to make the turtle move in a circle
        node.get_logger().info('Moving in the first circle...')
        rclpy.spin_once(node)
        revolutions_completed += 0.1                                        # Assuming a control rate of 10 Hz ,it will count the revolution



    # Stop the turtle when it completes one full circle
    twist1.linear.x = 0.0
    twist1.angular.z = 0.0
    publisher.publish(twist1)
    spawn_client = node.create_client(Spawn, '/spawn')



    while not spawn_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /spawn service...')


#Here the second turtle is spawn with some offset.

    spawn_request = Spawn.Request()
    spawn_request.x = 5.44445                                               # X-coordinate for the new turtle's spawn location
    spawn_request.y = 5.44445                                               # Y-coordinate for the new turtle's spawn location  (offset is given because of initial default spawn)
    spawn_request.name = 'turtle2'
    spawn_request.theta = 0.0                                               # Initial orientation (0 radians)
    spawn_future = spawn_client.call_async(spawn_request)
    
    
# The whole process of second bot is same as the first one. (same code with slight change in values)


    publisher = node.create_publisher(Twist, '/turtle2/cmd_vel', 10)

    
    twist2 = Twist()                                                        # Create a Twist message for moving in a circle
    twist2.linear.x = 3.0                                                   # Linear velocity (forward speed)
    twist2.angular.z = -2.0                                                 # Angular velocity (rotation rate)

    
    time_to_move = 10.0                                                     #Set the time to move in a circle (in seconds) and calculate the number of revolutions
    rate = node.create_rate(10)
    num_revolutions = time_to_move * twist2.angular.z / (2 * math.pi)
   

    
    revolutions_completed = math.pi                                         # Here final value is used as it will subtract the revolution from the intial to get it zero.
    while rclpy.ok() and revolutions_completed > num_revolutions:
        publisher.publish(twist2)                                           # Publish the Twist message to make the turtle move in a circle
        node.get_logger().info('Moving in second second circle...')
        rclpy.spin_once(node)
        revolutions_completed -= 0.1 
        
        
    twist2.linear.x = 0.0
    twist2.angular.z = 0.0
    publisher.publish(twist2)



    # Wait for the spawn request to complete
    rclpy.spin_until_future_complete(node, spawn_future)


    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()

