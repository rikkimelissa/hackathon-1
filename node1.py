#!/usr/bin/env python


import rospy
import roslib
from std_msgs.msg import String
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def node1cb(data, bridge ):
    #rospy.loginfo("new image")

    try:
       frame1 = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)

    hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
    

    #blur = cv2.GaussianBlur(frame1,(15,15),0)
   
    hmin = cv2.getTrackbarPos('HMin','filter')
    smin = cv2.getTrackbarPos('SMin','filter')
    vmin = cv2.getTrackbarPos('VMin','filter')

    hmax = cv2.getTrackbarPos('HMax','filter')
    smax = cv2.getTrackbarPos('SMax','filter')
    vmax = cv2.getTrackbarPos('VMax','filter')

    min_red_vales = np.array([hmin,smin,vmin])
    max_red_vales = np.array([hmax,smax,vmax])

    mask = cv2.inRange(hsv, min_red_vales, max_red_vales)
    kernel = np.ones((380,380),np.uint8)
    erosion = cv2.erode(mask,kernel,iterations = 1)#Erode
    kernel1 = np.ones((380,380),np.uint8)
    dilation = cv2.dilate(erosion,kernel1,iterations = 1)#Dilate
    
    circles = cv2.HoughCircles(dilation,cv.CV_HOUGH_GRADIENT,1,2 0,param1=50,param2=30,minRadius = 0, maxRadius=0)
    circles = np.uint16(np.around(circles))

    for i in circles[0,:]:
        cv2.circle(dilation,(i[0],i[1]),i[2],(0,255,0),2)
        cv2.circle(dilation,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow('frame1',frame1)
    cv2.imshow('dilation',mask)
    
    cv2.waitKey(25)

def nothing(self):
    pass

def node1():
    
    
    rospy.init_node('node1', anonymous=True)
    
    bridge = CvBridge()

    cv2.namedWindow ('filter')

    cv2.createTrackbar('HMin','filter',2,255,nothing)
    cv2.createTrackbar('SMin','filter',102,255,nothing)
    cv2.createTrackbar('VMin','filter',198,255,nothing)

    cv2.createTrackbar('HMax','filter',20,255,nothing)
    cv2.createTrackbar('SMax','filter',155,255,nothing)
    cv2.createTrackbar('VMax','filter',232,255,nothing)


    rospy.Subscriber('usb_cam/image_raw', Image, node1cb,bridge)
    
    rospy.spin()

if __name__ == '__main__':
    
    node1()

