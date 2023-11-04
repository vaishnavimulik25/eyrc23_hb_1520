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
import cv2
import argparse
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
        self.get_logger().info("cv image converted")        

        #Detect Aruco marker
        ap = argparser.ArgumentParser()
        ap.add_argument("-i","--image",required=True,help="path to input image containing Aruco tag")
        ap.add_argument("-t","--type",type=str,default="DICT_ARUCO_ORIGINAL",help="type of Aruco tag to detect")
        args = vars(ap.parse_args())
        # define names of each possible ArUco tag OpenCV supports
        ARUCO_DICT = {
        	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners,ids,rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
        
        self.get_logger().info("aruco detected")        
        # Publish the bot coordinates to the topic  /detected_aruco
        #self.aruco_publisher = self.create_publisher(Pose)


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
