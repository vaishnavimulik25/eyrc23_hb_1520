
#! /usr/bin/env python3

'''
*****************************************************************************************
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
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
import cv2
import math
#import argparse
from cv_bridge import CvBridge #for using cv_bridge for converting ros image to opencv image

# Import the required modules
##############################################################
class ArUcoDetector(Node,CvBridge):

    def __init__(self):
        super().__init__('ar_uco_detector')
        
        # Subscribe the topic /camera/image_raw
        self.camera_subscriber = self.create_subscription(Image,"/camera/image_raw",self.image_callback,10)

    def image_callback(self, msg):
        
        #convert ROS image to opencv image
        cvb = CvBridge()
        cv_image = cvb.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #self.get_logger().info("cv image converted")        

        #Detect Aruco marker
        #NOTE only for reference
       # ap = argparse.ArgumentParser()
       # ap.add_argument("-t","--type",type=str,default="DICT_ARUCO_ORIGINAL",help="type of Aruco tag to detect")
       # args = vars(ap.parse_args())
       # # define names of each possible ArUco tag OpenCV supports
       # ARUCO_DICT = {
       # 	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
       # 	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
       # 	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
       # 	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
       # 	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
       # 	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
       # 	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
       # 	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
       # 	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
       # 	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
       # 	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
       # 	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
       # 	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
       # 	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
       # 	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
       # 	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
       # 	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
       # 	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
       # 	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
       # 	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
       # 	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
       # }

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        (corners,ids,rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
        
        #self.get_logger().info("aruco detected successfully")        
        
        # Publish the bot coordinates to the topic  /detected_aruco
        (x1,y1) = corners[0][0][0][:2]
        (x2,y2) = corners[0][0][1][:2]
        (x3,y3) = corners[0][0][2][:2]
        (x4,y4) = corners[0][0][3][:2]
       
        x_centroid = (x1 + x2 + x3 + x4)/4
        y_centroid = (y1 + y2 + y3 + y4)/4
        theta = math.atan(y_centroid/x_centroid)
        
        #coordinates to be converted in msg type Pose2D 
        coordinates = Pose2D()
        coordinates.x = x_centroid
        coordinates.y = y_centroid
        coordinates.theta = theta
        #self.get_logger().info("coordinates of bot got successfully")        
        
        # Publish the bot coordinates to the topic  /detected_aruco
        (x11,y11) = corners[0][1][0][:2]
        (x21,y21) = corners[0][1][1][:2]
        (x31,y31) = corners[0][1][2][:2]
        (x41,y41) = corners[0][1][3][:2]
       
        x_centroid1 = (x11 + x21 + x31 + x41)/4
        y_centroid1 = (y11 + y21 + y31 + y41)/4
        theta1 = math.atan(y_centroid1/x_centroid1)
        
        #coordinates to be converted in msg type Pose2D 
        coordinates1 = Pose2D()
        coordinates1.x = x_centroid1
        coordinates1.y = y_centroid1
        coordinates1.theta = theta1
        #self.get_logger().info("coordinates of bot got successfully")        


        # Publish the bot coordinates to the topic  /detected_aruco
        (x12,y12) = corners[0][2][0][:2]
        (x22,y22) = corners[0][2][1][:2]
        (x32,y32) = corners[0][2][2][:2]
        (x42,y42) = corners[0][2][3][:2]
       
        x_centroid2 = (x12 + x22 + x32 + x42)/4
        y_centroid2 = (y12 + y22 + y32 + y42)/4
        theta2 = math.atan(y_centroid2/x_centroid2)
        
        #coordinates to be converted in msg type Pose2D 
        coordinates2 = Pose2D()
        coordinates2.x = x_centroid2
        coordinates2.y = y_centroid2
        coordinates2.theta = theta2
        #self.get_logger().info("coordinates of bot got successfully")        

        #created /detected_aruco_1 topic
        self.aruco_publisher = self.create_publisher(Pose2D,"/detected_aruco_1",10)
        self.aruco_publisher.publish(coordinates)
        #self.get_logger().info("coordinates published successfully")        

        #created /detected_aruco_2 topic
        self.aruco_publisher = self.create_publisher(Pose2D,"/detected_aruco_2",10)
        self.aruco_publisher.publish(coordinates1)
        
        #created /detected_aruco_3 topic
        self.aruco_publisher = self.create_publisher(Pose2D,"/detected_aruco_3",10)
        self.aruco_publisher.publish(coordinates2)
def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

