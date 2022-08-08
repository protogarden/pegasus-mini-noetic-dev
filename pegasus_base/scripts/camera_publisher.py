#!/usr/bin/env python

import rospy
import yaml
import cv2
import os
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import time
import numpy as np
from cv_bridge import CvBridge






rospy.init_node('camera_pub')
cam_port = 0
camera_type = "oak_d" #if using OAK-d camera, camera_type = "oak_d". If using standard webcam camera_type = "standard".

#cam_port = rospy.get_param("/port")
#camera_type = rospy.get_param("/cam_type")

cam_standard_info = CameraInfo()
cam_standard_info.K = [712.1036081760766, 0.0, 467.68377603664027, 0.0, 712.403233695187, 269.04193985198145, 0.0, 0.0, 1.0]
cam_standard_info.D = [0.021533613848944824, -0.09687445622248622, 0.002708932663089853, -0.004650287718603307, 0.0]
cam_standard_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
cam_standard_info.P = [703.6085815429688, 0.0, 461.8930612015138, 0.0, 0.0, 714.2974853515625, 270.0070402640449, 0.0, 0.0, 0.0, 1.0, 0.0]




# Create pipeline
if camera_type == "oak_d": 
    import depthai as dai



    pipeline = dai.Pipeline()

    # Define source and output
    camRgb = pipeline.create(dai.node.ColorCamera)
    xoutRgb = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")

    # Properties
    camRgb.setPreviewSize(640,360)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.preview.link(xoutRgb.input)
    #camera_information:
    cam_info = CameraInfo()
    cam_info.K = [712.1036081760766, 0.0, 467.68377603664027, 0.0, 712.403233695187, 269.04193985198145, 0.0, 0.0, 1.0]
    cam_info.D = [0.021533613848944824, -0.09687445622248622, 0.002708932663089853, -0.004650287718603307, 0.0]
    cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info.P = [703.6085815429688, 0.0, 461.8930612015138, 0.0, 0.0, 714.2974853515625, 270.0070402640449, 0.0, 0.0, 0.0, 1.0, 0.0]
    # Linking


    with dai.Device(pipeline) as device:
        print('Usb speed: ', device.getUsbSpeed().name)
        print('Connected cameras: ', device.getConnectedCameras())
        


        img_pub = rospy.Publisher('/camera_pub/image_rect', Image, queue_size = 10)
        cam_pub = rospy.Publisher('/camera_pub/camera_info', CameraInfo, queue_size = 10)
        

        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        rate = rospy.Rate(10)
        rospy.loginfo("/port: %s, /cam_type: %s", cam_port, camera_type)
        bridge = CvBridge()

        while not rospy.is_shutdown():
            inRgb = qRgb.get()
            img = inRgb.getCvFrame()
            #img =cv2.resize(img,(960,540))
            img_msg = Image()
            #cv2.imshow('window_name', img)
            
          

            stamp = rospy.Time.now()
            
            img_msg.height = img.shape[0]
            img_msg.width = img.shape[1]
            img_msg.step = img.strides[0]
            img_msg.encoding = 'bgr8'
            img_msg.header.frame_id = 'camera'
            img_msg.header.stamp = stamp
            #img_msg.data = bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.data = img.flatten().tolist()
   
            
            img_pub.publish(img_msg)
            

            cam_info.header.stamp = stamp
            rate.sleep()

            #publish the camera info messages first
            cam_pub.publish(cam_info)
        

            # Get BGR frame from NV12 encoded video frame to show with opencv
            # Visualizing the frame on slower hosts might have overhead

           
           

            if cv2.waitKey(1) == ord('q'):
                break


elif camera_type == "standard":
    vid = cv2.VideoCapture(cam_port)
    img_pub = rospy.Publisher('/camera_pub/image_rect', Image, queue_size = 10)
    cam_pub = rospy.Publisher('/camera_pub/camera_info', CameraInfo, queue_size = 10)
    rospy.init_node('camera_pub')
    rospy.loginfo("/port: %s, /cam_type: %s", cam_port, camera_type)

    while not rospy.is_shutdown():
        ret, frame = vid.read()
        img =cv2.resize(frame,(960,540))
        img_msg = Image()
        stamp = rospy.Time.now()
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.step = img.strides[0]
        img_msg.encoding = 'bgr8'
        img_msg.header.frame_id = 'camera'
        img_msg.header.stamp = stamp
        img_msg.data = img.flatten().tolist()
        
        img_pub.publish(img_msg)

        cam_standard_info.header.stamp = stamp
        rate.sleep()

        #publish the camera info messages first
        cam_pub.publish(cam_standard_info)
        cv2.imshow('frame', img)
    

        if cv2.waitKey(1) == ord('q'):
            break

else: 
    rospy.loginfo("No camera parameters received")



'''

Oak-D parameters with the above config: 

D = [0.021533613848944824, -0.09687445622248622, 0.002708932663089853, -0.004650287718603307, 0.0]
K = [712.1036081760766, 0.0, 467.68377603664027, 0.0, 712.403233695187, 269.04193985198145, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [703.6085815429688, 0.0, 461.8930612015138, 0.0, 0.0, 714.2974853515625, 270.0070402640449, 0.0, 0.0, 0.0, 1.0, 0.0]

'''


