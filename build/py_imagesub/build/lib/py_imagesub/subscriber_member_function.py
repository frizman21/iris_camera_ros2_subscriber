# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import time
import socket
# import exceptions
import math
import argparse
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


import cv2 # OpenCV library
import cv2 as cv
from cv2 import aruco

import numpy as np



id_to_find = 72
marker_size = 19 #cm
takeoff_height = 8

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# parameters = aruco.DetectorParameters_create()

# dictionary = cv2.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)



print("OpenCV Version: {}".format(cv2.__version__))
print("Python Version")
print(sys.version)

# dictionary = cv2.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv2.aruco.DetectorParameters()
# detector = cv2.aruco.ArucoDetector(dictionary, parameters)


##Camera
horizontal_res = 640
vertical_res = 480

horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

calib_path="/home/mike/ros2_opencv_lander/py_imagesub/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        print("Creating subscriber")
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.br = CvBridge()

    def listener_callback(self, msg):
                
        self.get_logger().info('I heard:')
        
        # if vehicle.mode!='LAND':
        #     # No-Op - if not landing, ignore this message
        #     self.get_logger().info('Not in landing mode')
        # else:

        current_frame = self.br.imgmsg_to_cv2(msg)

        gray_img = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)

        # Example content - replaced with new method for same task
        # corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        # corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        # markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray_img)
        
        # markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray_img)
        # (markerCorners, markerIds, rejectedCandidates) = aruco.detectMarkers(opencv_image_gray, self.aruco_dict)
    
    # LATEST
        corners, markerIds, rejectedCandidates = aruco.detectMarkers(gray_img, dictionary=aruco_dict)

        # (corners, detected_ids, rejected_image_points) = aruco.detectMarkers(opencv_image_gray, self.aruco_dict)
    
        if markerIds is not None and markerIds[0] == id_to_find:
            print('Found something')
            
            
            # OLD way
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)

            # C++ way
            # solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));




            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])
        
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)

            print(x_ang)
            print(y_ang)

        # if markerIds is not None:
        #     self.get_logger().info('   Found Something!')
        # else:
        #     self.get_logger().info('   Nothing.')

        cv2.imshow("camera", gray_img)
        cv2.waitKey(1)
        
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    print("destroying subscriber")
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
