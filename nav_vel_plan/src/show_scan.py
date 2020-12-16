#!/usr/bin/env python
'''demo_1 ROS Node'''
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import cv2
import math
from scipy.optimize import leastsq
from math import pi

dis_ranges = 2.0
angle_ranges = 10
angle_incrementself = 1.0
x_point = []
y_point = []

def residuals(p):
    a,b,r = p
    return r**2-(y_point-b)**2-(x_point-a)**2

def callback(data):
    #print(len(data.ranges))
    img = np.zeros((800,800,3) ,np.uint8)
    img[:,:] = [255,255,255]
    angle = data.angle_min
    lst = [i for i in data.ranges]

    
    for j in lst:
        if math.isinf(j) is True:
            j = 0

        x = -math.trunc((j*80.0)*math.cos(angle))
        y = -math.trunc((j*80.0)*math.sin(angle))
        if y > 380 or y < -380 or x < -380 or x>380:
            x = 0
            y = 0

        img[(x + 380):(x + 382), (y + 380) : (y + 382)] = [0,0,255]

        angle = angle + data.angle_increment

    cv2.imshow("laser",img)
    cv2.waitKey(1)

def listener():
    '''demo_1 Subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('demo_1', anonymous=True)

    rospy.Subscriber("robot2/scan",LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
