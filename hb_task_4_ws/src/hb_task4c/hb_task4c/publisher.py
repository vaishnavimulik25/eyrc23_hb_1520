import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
        # Initialze Publisher with the "/Integer" topic
        self.integer = self.create_publisher(Int32, "Integer", 10)
        self.cmd_vel_bot_1 = self.create_publisher(Twist, '/cmd_vel/bot1', 10)
        # self.cmd_vel_bot_2 = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.cmd_vel_bot_3 = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def vel_pub_bot_1(self):
        bot1_vel = Twist()
        bot1_vel.linear.x = 1.0
        bot1_vel.linear.y = 1.0
        bot1_vel.angular.z = 0.0
        self.cmd_vel_bot_1.publish(bot1_vel)

    def timer_callback(self):
        msg = Int32()
        # Assign the msg variable to i
        msg.data = self.i
        # self.i = msg.data
        self.integer.publish(msg)
        # Publish the msg
        self.i = self.i + 1
        # Increment the i
        self.vel_pub_bot_1()



def main(args=None):
    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
