#!/usr/bin/env python2

import rospy
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist             ## use this message type to move mobile robot
from sensor_msgs.msg import CompressedImage     ## use this to sub & pub the videodata


rospy.init_node('cam_frame',anonymous=True)
pub = rospy.Publisher('frame',CompressedImage,queue_size = 1000)

def video_processing(ros_data):
    ####<<< Decode Ros_data to the frame >>>####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    setting_bar()
    while True:
        H_MAX = cv2.getTrackbarPos('H_MAX', 'HSV_settings')
        H_MIN = cv2.getTrackbarPos('H_MIN', 'HSV_settings')
        S_MAX = cv2.getTrackbarPos('S_MAX', 'HSV_settings')
        S_MIN = cv2.getTrackbarPos('S_MIN', 'HSV_settings')
        V_MAX = cv2.getTrackbarPos('V_MAX', 'HSV_settings')
        V_MIN = cv2.getTrackbarPos('V_MIN', 'HSV_settings')
        lower = np.array([H_MIN, S_MIN, V_MIN])
        higher = np.array([H_MAX, S_MAX, V_MAX])
        hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
        Gmask = cv2.inRange(hsv, lower, higher)
        G = cv2.bitwise_and(image_np, image_np, mask = Gmask)
        cv2.imshow('cam_load',image_np)
        cv2.imshow('G',G)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

def onChange(x):
    pass

def setting_bar():

    cv2.namedWindow('HSV_settings')
    cv2.createTrackbar('H_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('H_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('H_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('H_MIN', 'HSV_settings', 0)
    cv2.createTrackbar('S_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('S_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('S_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('S_MIN', 'HSV_settings', 0)
    cv2.createTrackbar('V_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('V_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('V_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('V_MIN', 'HSV_settings', 0)


if __name__ == '__main__':
    #print("working?")
    sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, video_processing,queue_size = 1000)
    rospy.spin()
