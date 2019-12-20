#!/usr/bin/env python

import cv2, time
import numpy as np
import pickle

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
cvbridge = CvBridge()
def cfg(data):
    global cvbridge
    cam_img = cvbridge.imgmsg_to_cv2(data, 'bgr8')
    cv2.imshow("1", cam_img)
    cv2.waitKey(1)

def nothing(val):
    pass

if True:#__name__ == "__main__":
    rospy.init_node("configure")
    rospy.Subscriber("/usb_cam/image_raw", Image, cfg)
    rate = rospy.Rate(10)
    '''
    cv2.namedWindow('video')
    cv2.createTrackbar('TLx', 'video', 0, 640, nothing)
    cv2.createTrackbar('TLy', 'video', 0, 480, nothing)
    cv2.createTrackbar('TRx', 'video', 0, 640, nothing)
    cv2.createTrackbar('TRy', 'video', 0, 480, nothing)
    cv2.createTrackbar('LLx', 'video', 0, 640, nothing)
    cv2.createTrackbar('LLy', 'video', 0, 480, nothing)
    cv2.createTrackbar('LRx', 'video', 0, 640, nothing)
    cv2.createTrackbar('LRy', 'video', 0, 480, nothing)
    cv2.createTrackbar('blur', 'video', 0, 300, nothing)
    cv2.createTrackbar('lth', 'video', 0, 300, nothing)
    cv2.createTrackbar('uth', 'video', 0, 300, nothing)
    '''
    data = []
    try:
        with open('cfg', 'rb') as f:
            data = pickle.load(f)
    except Exception:
        data = [(0, 0), (640, 0), (0, 480), (640, 480), 5, 60, 70]
    print(data)
    '''
    cv2.setTrackbarPos('TLx', 'video', data[0][0])
    cv2.setTrackbarPos('TLy', 'video', data[0][1])
    cv2.setTrackbarPos('TRx', 'video', data[1][0])
    cv2.setTrackbarPos('TRy', 'video', data[1][1])
    cv2.setTrackbarPos('LLx', 'video', data[2][0])
    cv2.setTrackbarPos('LLy', 'video', data[2][1])
    cv2.setTrackbarPos('LRx', 'video', data[3][0])
    cv2.setTrackbarPos('LRy', 'video', data[3][1])
    cv2.setTrackbarPos('blur', 'video', data[4])
    cv2.setTrackbarPos('lth', 'video', data[5])
    cv2.setTrackbarPos('uth', 'video', data[6])
    '''
    while not rospy.is_shutdown():
        rate.sleep()