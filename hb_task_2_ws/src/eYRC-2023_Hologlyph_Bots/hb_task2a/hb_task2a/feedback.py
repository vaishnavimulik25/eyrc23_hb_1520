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
import argparse
# for using cv_bridge for converting ros image to opencv image
from cv_bridge import CvBridge

# Import the required modules
##############################################################


class ArUcoDetector(Node, CvBridge):

    def __init__(self):
        super().__init__('ar_uco_detector')

        # Subscribe the topic /camera/image_raw
        self.camera_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)
        self.x_centroid = 0.0
        self.y_centroid = 0.0
        self.theta = 0.0
        self.coordinates = Pose2D()

    def image_callback(self, msg):

        # convert ROS image to opencv image
        cvb = CvBridge()
        cv_image = cvb.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.get_logger().info("cv image converted")

        # Detect Aruco marker
        # NOTE only for reference
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
        # arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoDict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            cv_image, arucoDict, parameters=arucoParams)
        corner_copy = corners
        for (corner, ID) in zip(corners, ids):
            if ID == 1:

                (x1, y1) = corner[0][0][:2]
                (x2, y2) = corner[0][1][:2]
                (x3, y3) = corner[0][2][:2]
                (x4, y4) = corner[0][3][:2]
                self.x_centroid = (x1 + x2 + x3 + x4)/4
                # print(y1,y2,y3,y4)
                self.y_centroid = (y1 + y2 + y3 + y4)/4
                self.theta = math.atan((y2-y1)/(x2-x1))

                self.coordinates.x = self.x_centroid
                self.coordinates.y = self.y_centroid
                self.coordinates.theta = self.theta
                print(self.coordinates.x, self.coordinates.y,
                      self.coordinates.theta)

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
        cv2.putText(cv_image, str(self.coordinates.x),
                    (250, 250), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        cv2.putText(cv_image, str(self.coordinates.y),
                    (250, 270), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        cv2.putText(cv_image, str(self.coordinates.theta),
                    (250, 300), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        # created /detected_aruco topic
        self.aruco_publisher = self.create_publisher(
            Pose2D, "/detected_aruco", 10)
        self.aruco_publisher.publish(self.coordinates)
        # self.get_logger().info("coordinates published successfully")

        cv2.imshow("Camera output resized", cv_image)


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
