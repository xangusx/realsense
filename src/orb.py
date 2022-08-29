#!/usr/bin/python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import String,Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


def callback(data):

    global bridge
    bridge = CvBridge()
    # pub = rospy.Publisher('/distance',Int64,queue_size=1)
    rate = rospy.Rate(10)
    depth_image = bridge.imgmsg_to_cv2(data,'16UC1')
    gray = depth_image/32767*255

    orb = cv2.ORB_create()

    kp1 = orb.detect(gray)
    kp2 = orb.detect()
    kp1, des1 = orb.compute(gray, kp1)
    kp2, des1 = orb.compute()

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matcher = bf.match(des1, des2)

    
    
    


def get_distance():
    rospy.init_node('detect_harris_features',anonymous=True,disable_signals = False)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,callback)
    rospy.spin()

if __name__=='__main__':
    global fx,fy,ppx,ppy

    first = True
    #K = [fx 0 ppx；0 fy ppy；0 0 1]
    fx = 609.68017578125
    fy = 608.42529296875
    ppx = 321.38134765625
    ppy = 238.43679809570

    get_distance()