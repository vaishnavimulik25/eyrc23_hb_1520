#!/usr/bin/env python3


from cv_bridge import CvBridge
import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal
import numpy as np

class HBController1(Node):
    def __init__(self):
        # initalise node
        super().__init__('hb_controller1')

        # pose params
        self.x=0
        self.y=0
        self.theta=0

        # bot params
        self.d=0.68
        self.r=0.14

        # threshold params
        self.linear_thresh=1
        self.ang_thresh=4*0.01745

        # goals
        self.x_goals=[]
        self.y_goals=[]
        self.theta_goals=0.0
        self.prev=[]


        # subscriber/publisher
        self.goal_sub = self.create_subscription(Goal,'hb_bot_1/goal',self.task2_goals_Cb,10)
        self.aruco_subscriber = self.create_subscription(
            Pose2D, '/detected_aruco_1', self.aruco_feedback_Cb, 10)
        self.left_wheel_pub = self.create_publisher(
            Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.right_wheel_pub = self.create_publisher(
            Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.front_wheel_pub = self.create_publisher(
            Wrench, '/hb_bot_1/rear_wheel_force', 10)
        
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        
        # pid params
        self.params_linear={'Kp':0.08, 'Ki':0, 'Kd':0}
        self.params_ang={'Kp':0.0, 'Ki':0, 'Kd':0}
        self.intg=0
        self.last_error=0

        # ROS msg
        self.rw_msg=Wrench()
        self.lw_msg=Wrench()
        self.fw_msg=Wrench()

    def task2_goals_Cb(self, msg):
        self.x_goals.clear()
        self.y_goals.clear()
        self.theta_goals = 0.0

        self.x_goals = msg.x
        self.y_goals = msg.y
        self.theta_goals = 0.0

    def aruco_feedback_Cb(self, msg):
        self.x=msg.x
        self.y=msg.y
        self.theta=0.0

    def inverse_kinematics(self, vx, vy, w):
        # inverse kinematic conversion for each wheel
        u1=(-self.d*w+vx)/self.r
        u2=(-self.d*w-vx/2-math.sqrt(3)*vy/2)/self.r
        u3=(-self.d*w-vx/2+math.sqrt(3)*vy/2)/self.r

#        self.matrix_3x3 = -np.array([[7.145919679862797, -12.376237623762377, -4.8615170766646765], [7.145919679862797, 12.376237623762377, -4.8615170766646765], [-14.291839, 0.0, -4.861517076]])
#        self.vector_3x1 = np.array([vx, vy, w])
#        self.result = np.dot(self.matrix_3x3, self.vector_3x1)
#        u1,u2,u3 = self.result
        return u2, u3, u1

    def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['Kp'] * prop + const['Ki'] * self.intg + const['Kd'] * diff
        self.last_error = error

        return balance

    # angular pid function
    def getAngVel(self, error, const):
        ang_vel=0

        if abs(error) > self.ang_thresh:
            if error > 3.14:
                ang_vel = self.pid((error-6.28), const)
            elif error < -3.14:
                ang_vel = self.pid((error+6.28), const)
            else:
                ang_vel = self.pid(error, const)

            if ang_vel<0: ang_vel=-1.5
            else: ang_vel=1.5

        else:
            self.stop()

        return ang_vel

    # linear pid function
    def getLinearVel(self, error_x,  error_y, const, x=True):
        v_x=0.0
        v_y=0.0
        
        if abs(error_x)>self.linear_thresh or abs(error_y)>self.linear_thresh:
            v_x=self.pid(error_x, const)
            v_y=self.pid(error_y, const)
        else:
            self.stop()

        return v_x, v_y

    # bot halt function
    def stop(self):
        self.fw_msg.force.x=0.0
        self.rw_msg.force.x=0.0
        self.lw_msg.force.x=0.0
        
        self.front_wheel_pub.publish(self.fw_msg)
        self.right_wheel_pub.publish(self.rw_msg)
        self.left_wheel_pub.publish(self.lw_msg)

def main(args=None):
    rclpy.init(args=args)
    hb_controller1 = HBController1()
    
    # control loop
    while rclpy.ok():
        #if hb_controller1.prev==hb_controller1.theta_goals: continue

        for i in range(len(hb_controller1.x_goals)):
            goal_x=hb_controller1.x_goals[i]
            goal_y=hb_controller1.y_goals[i]  #conversion to cartesian system
            goal_theta=hb_controller1.theta_goals

            while True:
                # error calculation
                angle_error=goal_theta-hb_controller1.theta
                error_x=(goal_x-hb_controller1.x)*math.cos(hb_controller1.theta)+(goal_y-hb_controller1.y)*math.sin(hb_controller1.theta)
                error_y=-(goal_x-hb_controller1.x)*math.sin(hb_controller1.theta)+(goal_y-hb_controller1.y)*math.cos(hb_controller1.theta)

                # velocity calculation
                v_x, v_y=hb_controller1.getLinearVel(error_x,  error_y, hb_controller1.params_linear)
                ang_vel=hb_controller1.getAngVel(angle_error, hb_controller1.params_ang)

                # effort calculation
                fw_vel, rw_vel, lw_vel=hb_controller1.inverse_kinematics(v_x, v_y, ang_vel)
                hb_controller1.fw_msg.force.x=fw_vel
                hb_controller1.rw_msg.force.x=rw_vel
                hb_controller1.lw_msg.force.x=lw_vel

                # force publishing
                hb_controller1.front_wheel_pub.publish(hb_controller1.fw_msg)
                hb_controller1.right_wheel_pub.publish(hb_controller1.rw_msg)
                hb_controller1.left_wheel_pub.publish(hb_controller1.lw_msg)


                # move to next pose when reached target pose
                if abs(angle_error)<=hb_controller1.ang_thresh and abs(error_x)<=hb_controller1.linear_thresh and abs(error_y)<=hb_controller1.linear_thresh:
                   break
                
        hb_controller1.prev=hb_controller1.theta_goals
        rclpy.spin_once(hb_controller1)       

    rclpy.spin(hb_controller1)
    hb_controller1.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
