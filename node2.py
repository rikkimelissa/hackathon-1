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
from std_msgs.msg import UInt16

pantarget = 1500
tilttarget = 1500

def node2cb(data, extra_args):

    ser,var = extra_args
    x = data.data
    print x

    global pantarget
    global tilttarget

    if var == 'x':
        if x >= 340 and pantarget < 1950:
            #increase pan target by 
            pantarget += 10
        elif x <= 300 and pantarget > 1050:
            pantarget -= 10
    if var == 'y':
        if x >= 260 and tilttarget < 1950:
            #increase pan target by 
            tilttarget -= 10
        elif x <= 220 and tilttarget > 1050:
            tilttarget += 10
            
    
    #tiltpos = bytearray([132,0,x,x])    
    #panpos = bytearray([132,1,x,x])

    #paninput = int(x)
    #tiltinput = int(raw_input("enter tilt value> "))

    pancom = pantarget*4 & 0x7f
    pancom2 = (pantarget*4 >> 7) & 0x7f

    tiltcom = tilttarget*4 & 0x7f
    tiltcom2 = (tilttarget*4 >>7) & 0x7f

    #print pancom, pancom2
    #print tiltcom, tiltcom2

    panpos = bytearray([132,0,pancom,pancom2])
    tiltpos = bytearray([132,1,tiltcom,tiltcom2])

    ser.write(panpos)
    ser.write(tiltpos)



def node2():
    
    rospy.init_node('node2', anonymous=True)

    ser = serial.Serial('/dev/ttyACM0')

    initialpan = bytearray([132,1,112,46])
    initialtilt = bytearray([132,0,112,46])

    ser.write(initialpan)
    ser.write(initialtilt)
    
    rospy.Subscriber('node1pub', UInt16, node2cb,(ser,"x"))
    rospy.Subscriber('node1pub2', UInt16, node2cb,(ser,"y"))
    
    rospy.spin()

if __name__ == '__main__':
    
    node2()



