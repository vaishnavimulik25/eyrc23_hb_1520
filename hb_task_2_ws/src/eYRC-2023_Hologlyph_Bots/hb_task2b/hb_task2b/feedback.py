
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
import numpy as np
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
        self.x_centroid = 0.0
        self.x_centroid1 = 0.0
        self.x_centroid2 = 0.0
        self.y_centroid = 0.0
        self.y_centroid1 = 0.0
        self.y_centroid2 = 0.0
        self.theta = 0.0
        self.theta1 = 0.0
        self.theta2 = 0.0

    def image_callback(self, msg):
        
        #convert ROS image to opencv image
        cvb = CvBridge()
        cv_image = cvb.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #self.get_logger().info("cv image converted")        

        #Detect Aruco marker
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        (corners,ids,rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
        
        #self.get_logger().info("aruco detected successfully")        
        
        # Publish the bot coordinates to the topic  /detected_aruco1
        for (Corners,ID) in zip(corners,ids):
            if ID == 1:
                (x1,y1) = Corners[0][0][:2]
                (x2,y2) = Corners[0][1][:2]
                (x3,y3) = Corners[0][2][:2]
                (x4,y4) = Corners[0][3][:2]
       
                self.x_centroid,self.y_centroid = [(x1 + x2 + x3 + x4)/2,(y1 + y2 + y3 + y4)/2]
                self.xrm_centroid,self.yrm_centroid = [(x3 + x4)/2,(y3 + y4)/2]
#                self.theta = math.atan((self.y_centroid - self.yrm_centroid)/(self.x_centroid - self.xrm_centroid))
                self.theta = math.atan((y2-y1)/(x2-x1))
        
            if ID == 2: 
                # Publish the bot coordinates to the topic  /detected_aruco2
                (x11,y11) = Corners[0][0][:2]
                (x21,y21) = Corners[0][1][:2]
                (x31,y31) = Corners[0][2][:2]
                (x41,y41) = Corners[0][3][:2]
       
                self.x_centroid1,self.y_centroid1 = [(x11 + x21 + x31 + x41)/2,(y11 + y21 + y31 + y41)/2]
                self.xrm_centroid1,self.yrm_centroid1 = [(x31 + x41)/2,(y31 + y41)/2]
                if x11 != x21:
                    self.theta1 = math.atan((y21-y11)/(x21-x11))
                print(x11,x21,x31,x41)
#                self.theta1 = math.atan((self.y_centroid1 -self.yrm_centroid1)/(self.x_centroid1 - self.xrm_centroid1))
        
            if ID == 3:
                # Publish the bot coordinates to the topic  /detected_aruco3
                (x12,y12) = Corners[0][0][:2]
                (x22,y22) = Corners[0][1][:2]
                (x32,y32) = Corners[0][2][:2]
                (x42,y42) = Corners[0][3][:2]
       
                self.x_centroid2,self.y_centroid2 = [(x12 + x22 + x32 + x42)/2,(y12 + y22 + y32 + y42)/2]
                self.xrm_centroid2,self.yrm_centroid2 = [(x32 + x42)/2,(y32 + y42)/2]
                #self.theta2 = math.atan((self.y_centroid2 - self.yrm_centroid2)/(self.x_centroid2 - self.xrm_centroid2))
                self.theta2 = math.atan((y22-y12)/(x22-x12))
        
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUcomarker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(cv_image, str(
                    markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.waitKey(1)
        #coordinates to be converted in msg type Pose2D 
        coordinates = Pose2D()
        coordinates.x = self.x_centroid
        coordinates.y = self.y_centroid
        coordinates.theta = self.theta
        #self.get_logger().info("coordinates of bot got successfully")        
        
        #coordinates to be converted in msg type Pose2D 
        coordinates1 = Pose2D()
        coordinates1.x = self.x_centroid1
        coordinates1.y = self.y_centroid1
        coordinates1.theta = self.theta1
        #self.get_logger().info("coordinates of bot got successfully")        
        
        #coordinates to be converted in msg type Pose2D 
        coordinates2 = Pose2D()
        coordinates2.x = self.x_centroid2
        coordinates2.y = self.y_centroid2
        coordinates2.theta = self.theta2
        #self.get_logger().info("coordinates of bot got successfully")        

        #for hb_bot1
        cv2.putText(cv_image, str(coordinates.x),
                    (100, 250), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        cv2.putText(cv_image, str(coordinates.y),
                    (100, 270), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        #for hb_bot2
        cv2.putText(cv_image, str(coordinates1.x),
                    (250, 250), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        cv2.putText(cv_image, str(coordinates1.y),
                    (250, 270), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        
        #for hb_bot3
        cv2.putText(cv_image, str(coordinates2.x),
                    (400, 250), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        cv2.putText(cv_image, str(coordinates2.y),
                    (400, 270), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        #created /detected_aruco_1 topic
        self.aruco_publisher = self.create_publisher(Pose2D,'/detected_aruco_1',10)
        self.aruco_publisher.publish(coordinates)
        #self.get_logger().info("coordinates published successfully")        

        #created /detected_aruco_2 topic
        self.aruco_publisher = self.create_publisher(Pose2D,'/detected_aruco_2',10)
        self.aruco_publisher.publish(coordinates1)
        
        #created /detected_aruco_3 topic
        self.aruco_publisher = self.create_publisher(Pose2D,'/detected_aruco_3',10)
        self.aruco_publisher.publish(coordinates2)
        cv2.imshow("Camera output resized", cv_image)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

