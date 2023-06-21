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
print("OpenCV Version: {}".format(cv2.__version__))

import numpy as np


import gi
# import cv2
# import argparse
# import cv2 as cv

# import required library like Gstreamer and GstreamerRtspServer
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject
print("GstRtspServer Loaded")

print("Python Version")
print(sys.version)

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
from pymavlink import mavutil
print("DroneKit Loaded")

id_to_find = 72
marker_size = 19 #cm
takeoff_height = 8

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
# parameters = aruco.DetectorParameters_create()

# dictionary = cv2.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)


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

vehicle = connect('tcp:127.0.0.1:5762')
print(vehicle.mode)


def send_land_message(x,y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()

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
                
        self.get_logger().info('Received an image')
        
        # if vehicle.mode!='LAND':
        #     # No-Op - if not landing, ignore this message
        #     self.get_logger().info('Not in landing mode')
        # else:

        current_frame = self.br.imgmsg_to_cv2(msg)
        gray_img = cv2.cvtColor(current_frame,cv2.COLOR_BGR2GRAY)    
        corners, markerIds, rejectedCandidates = aruco.detectMarkers(gray_img, dictionary=aruco_dict)

    
        if markerIds is not None and markerIds[0] == id_to_find:
            print('  Found something')
            
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])

            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])

            print("  MARKER POSITION: x=" +x+" y= "+y+" z="+z)
        
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
            
            print("  X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))

            
            print("  Sending correction message to ArduPilot - " + str(x_ang) + "," + str(y_ang))
            send_land_message(x_ang,y_ang)
            print("    Sent")

        cv2.imshow("camera", gray_img)
        cv2.waitKey(1)


print("about to load Sensor Factory")

# Sensor Factory class which inherits the GstRtspServer base class and add
# properties to it.
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.number_frames = 0
        self.fps = 5
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
        self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                             'caps=video/x-raw,format=BGR,width={},height={},framerate={}/1 ' \
                             '! videoconvert ! video/x-raw,format=I420 ' \
                             '! x264enc speed-preset=ultrafast tune=zerolatency ' \
                             '! rtph264pay config-interval=1 name=pay0 pt=96' \
                             .format(640, 480, self.fps)
    # method to capture the video feed from the camera and push it to the
    # streaming buffer.
    def on_need_data(self, src, length):
        # if self.cap.isOpened():
        if True:
            # ret, frame = self.cap.read()

            filename = "videotestsrc-frame.jpg"
            ret = True
            frame = cv.imread(filename)
            # cv2.imshow("RTSP View", frame)

            if ret:
                # It is better to change the resolution of the camera 
                # instead of changing the image shape as it affects the image quality.
                frame = cv2.resize(frame, (640, 480), \
                    interpolation = cv2.INTER_LINEAR)
                data = frame.tostring()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.duration = self.duration
                timestamp = self.number_frames * self.duration
                buf.pts = buf.dts = int(timestamp)
                buf.offset = timestamp
                self.number_frames += 1
                retval = src.emit('push-buffer', buf)
                print('pushed buffer, frame {}, duration {} ns, durations {} s'.format(self.number_frames,
                                                                                       self.duration,
                                                                                       self.duration / Gst.SECOND))
                if retval != Gst.FlowReturn.OK:
                    print(retval)
    # attach the launch string to the override method
    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)
    
    # attaching the source element to the rtsp media
    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)

print("about to load Gst Server")

parser2 = argparse.ArgumentParser()
# parser2.add_argument("--device_id", required=True, help="device id for the \
#                 video device or video file location")
# parser2.add_argument("--fps", required=True, help="fps of the camera", type = int)
# parser2.add_argument("--image_width", required=True, help="video frame width", type = int)
# parser2.add_argument("--image_height", required=True, help="video frame height", type = int)
parser2.add_argument("--port", default=8554, help="port to stream video", type = int)
parser2.add_argument("--stream_uri", default = "/video_stream", help="rtsp video stream uri")
opt2 = parser2.parse_args()

print(str(opt2.port) + " going to " + opt2.stream_uri)

# Rtsp server implementation where we attach the factory sensor with the stream uri
class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.factory = SensorFactory()
        self.factory.set_shared(True)
        self.set_service(opt2.port)
        self.get_mount_points().add_factory(opt2.stream_uri, self.factory)
        self.attach(None)    


def main(args=None):

    # GObject.threads_init()
    # Gst.init(None)
    # server = GstServer()
    # loop = GObject.MainLoop()
    # loop.run()

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