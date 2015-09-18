#!/usr/bin/env python


import rospy
import roslib
from std_msgs.msg import String
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import serial




def node2cb(data):
    
    #tiltpos = bytearray([132,0,x,x])    
    #panpos = bytearray([132,1,x,x])

    paninput = int(raw_input("enter pan value> "))
    tiltinput = int(raw_input("enter tilt value> "))

    pancom = paninput*4 & 0x7f
    pancom2 = (paninput*4 >> 7) & 0x7f

    tiltcom = tiltinput*4 & 0x7f
    tiltcom2 = (tiltinput*4 >>7) & 0x7f

    #print pancom, pancom2
    #print tiltcom, tiltcom2

    panpos = bytearray([132,1,pancom,pancom2])
    tiltpos = bytearray([132,0,tiltcom,tiltcom2])

    ser.write(panpos)
    ser.write(tiltpos)



def node2():
    
    rospy.init_node('node2', anonymous=True)

    ser = serial.Serial('/dev/ttyACM0')

    rospy.Subscriber('node1', Image, node2cb)
    
    rospy.spin()

if __name__ == '__main__':
    
    node2()



