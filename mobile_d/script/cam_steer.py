#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist             ## use this message type to move mobile robot
from sensor_msgs.msg import CompressedImage     ## use this to sub & pub the videodata
rospy.init_node('cam')
pub = rospy.Publisher('cmd_vel',Twist)
global angular
global turn_speed
angular =0
turn_speed =0

class Detection:

    def __init__(self):
        self.Color_HSV = {}
        # hsv 3 low , hsv 3 high
        self.Color_HSV['RED'] = [0, 255, 20, 255, 255, 115]
        self.Color_HSV['BLUE'] = [81, 110, 0, 255, 255, 255]
        self.Color_HSV['YELLOW'] = [6, 27, 55, 32, 255, 156]
        return

    def line_detection(self, frame, color):

        lowerBound = np.array(self.Color_HSV[color][:3])
        upperBound = np.array(self.Color_HSV[color][3:6])

        # Color(Yellow) detecting
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        Gmask = cv2.inRange(hsv, lowerBound, upperBound)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        ret, thr = cv2.threshold(Gmask, 127, 255, 0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        p1_x = 0
        p1_y = 0
        p2_x = 0
        p2_y = 0
        angle = 0
        c_x = 0
        c_y = 0
        if len(contours) > 0:
            for i in range(len(contours)):
                # Get area value
                area = cv2.contourArea(contours[i])
                if area > 100:  # minimum yellow area
                    rect = cv2.minAreaRect(contours[i])
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    box = sorted(box, key=lambda box: box[1])
                    # print box
                    p1_x = (box[0][0] + box[1][0]) / 2
                    p1_y = (box[0][1] + box[1][1]) / 2
                    p2_x = (box[2][0] + box[3][0]) / 2
                    p2_y = (box[2][1] + box[3][1]) / 2
                    center_x = p1_x
                    center_y = p1_y
                    print 'p1x:',center_x,'p2x:',center_y + 50
                    ############################################################
                    def slope(vx1, vx2, vy1, vy2):  # Parameters to calculate slope
                        if vx2 - vx1 == 0:
                            return 0
                        else:
                            m = float(vy2 - vy1) / float(vx2 - vx1)  # Slope equation
                            theta1 = math.atan(m)  # calculate the slope angle
                            return theta1 * (180 / np.pi)
                    ############################################################
                    ha2 = slope(p1_x, p2_x, p1_y, p2_y)
                    cv2.line(frame, (p1_x, p1_y), (p2_x, p2_y), (0, 255, 0), 2)
                    cv2.line(frame, (p2_x, 0), (p2_x, 240), (0, 0, 255), 2)
                    cv2.circle(frame, (box[0][0], box[0][1]), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (box[1][0], box[1][1]), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (box[2][0], box[2][1]), 3, (0, 0, 255), -1)
                    cv2.circle(frame, (box[3][0], box[3][1]), 3, (0, 0, 255), -1)
                    #cv2.putText(frame, 'hello', (center_x, center_y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 2)
                    # c_x = box[1][0]
                    # print 'c_x:',c_x, 'c_y:',c_y
                    if ha2 > 0:
                        ha2 = 90 - ha2
                    elif ha2 < 0:
                        ha2 = -90 - ha2
                    angle = int(ha2)
                    # print angle
                return 1, angle, p2_x
        else:  # need to send a serial eventhough there is no yellow
            return 0, 0, 0

def move(linear,angular):
    twist = Twist()
    twist.linear.x = linear 
    twist.angular.z = angular
    pub.publish(twist)

def angular_change(x,angular_v):
    angular_s=0
    angular_s2=0
    middle_x = 400
    Kp=3
    Ki=0.001
    p_val=0
    i_val=0
    err_p = (x-middle_x)*1000/middle_x
    print "err_p",err_p
    P_val = angular_v-err_p*Kp
    i_val =i_val-err_p*Ki
    angular_s = p_val+i_val
    print "angular_s",angular_s
    return angular_s

def video_processing(ros_data):
    global angular
    global turn_speed
    ####<<< Decode Ros_data to the frame >>>####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #subimg = image_np[200:600,0:400]
    line_detect = Detection() 
    ret, l_angle,mid_x = line_detect.line_detection(image_np,'BLUE')
    print l_angle,angular
    turn_speed = angular_change(mid_x,angular)
    linear = -0.3
    angular = turn_speed
    cv2.imshow('image_np',image_np)
    cv2.waitKey(1) & 0xFF
    move(linear,angular)


if __name__ == '__main__':
    sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, video_processing)
    rospy.spin()