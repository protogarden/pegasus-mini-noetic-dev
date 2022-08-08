#!/usr/bin/env python3

import rospy
import yaml
import cv2
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate = 5,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def pub_camera():
    rospy.init_node('camera_pub')
    camera_framerate = rospy.get_param('~framerate', 5)
    camera_pub_topic = rospy.get_param('~camera_image_topic', '/camera_pub/image_rect')
    camera_info_topic = rospy.get_param('~camera_info_topic', '/camera_pub/camera_info')
    capture_width = rospy.get_param('~capture_width', 1280)
    capture_height = rospy.get_param('~capture_height', 720)
    pub_width = rospy.get_param('~image_pub_width', 640)
    pub_height = rospy.get_param('~image_pub_height', 360)
    camera_K = rospy.get_param('~cam_K', [673.2482675259557, 0.0, 329.80724044927996, 0.0, 675.6751413943377, 180.66056825004506, 0.0, 0.0, 1.0])
    camera_D = rospy.get_param('~cam_D', [0.21887280107995002, -0.4072944753663401, -0.00013698773470533307, 0.0009668096667496951, 0.0])
    camera_R = rospy.get_param('~cam_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    camera_P = rospy.get_param('~cam_P', [693.0473022460938, 0.0, 330.2610844036044, 0.0, 0.0, 696.1262817382812, 180.62339938821424, 0.0, 0.0, 0.0, 1.0, 0.0])



    cam_standard_info = CameraInfo()
    cam_standard_info.K = camera_K
    cam_standard_info.D = camera_D
    cam_standard_info.R = camera_R
    cam_standard_info.P = camera_P
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2,framerate=camera_framerate, capture_width = capture_width, capture_height = capture_height,display_width=pub_width,display_height=pub_height), cv2.CAP_GSTREAMER)
    img_pub = rospy.Publisher(camera_pub_topic, Image, queue_size = 10)
    cam_pub = rospy.Publisher(camera_info_topic, CameraInfo, queue_size = 10)
    bridge = CvBridge()
    


    fps = video_capture.get(cv2.CAP_PROP_FPS)
    print("FPS",fps)
    rate = rospy.Rate(fps)
  
    
    

    if video_capture.isOpened():

        while not rospy.is_shutdown():
            ret_val, frame = video_capture.read()
            #img = cv2.resize(frame,(960,540))

            #frame =cv2.resize(frame,(960,540))
            #cv2.imshow('frame', frame)
  
            stamp = rospy.Time.now()
            img_msg = bridge.cv2_to_imgmsg(frame, "rgb8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera'
            
            
            

            cam_standard_info.header.stamp = stamp
            

            #publish the camera info messages first
            cam_pub.publish(cam_standard_info)
            img_pub.publish(img_msg)
            
            rate.sleep()

 
           
        

           

    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    pub_camera()

'''
D = [0.21887280107995002, -0.4072944753663401, -0.00013698773470533307, 0.0009668096667496951, 0.0]
K = [673.2482675259557, 0.0, 329.80724044927996, 0.0, 675.6751413943377, 180.66056825004506, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [693.0473022460938, 0.0, 330.2610844036044, 0.0, 0.0, 696.1262817382812, 180.62339938821424, 0.0, 0.0, 0.0, 1.0, 0.0]
'''

