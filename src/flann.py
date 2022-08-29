#!/usr/bin/python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
import pysift
from matplotlib import pyplot as plt
from std_msgs.msg import String,Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

global first
first = True

def callback(data):
    global bridge
    global gray1,kp1,des1
    global gray2,kp2,des2

    bridge = CvBridge()
    # pub = rospy.Publisher('/distance',Int64,queue_size=1)
    rate = rospy.Rate(10)
    depth_image = bridge.imgmsg_to_cv2(data,'16UC1')
    gray = depth_image/32767*255

    sift = cv2.SIFT.create()
    gray1 = gray
    kp1 = sift.detect(gray1,None)
    des1 = sift.comput(gray1,kp1)

    if first:
        # gray2=gray
        # kp2 = kp1
        des2 = des1
        first = -first 
    else:
        bf = cv2.FlannBasedMatcher()
        matcher = bf.knnMatch(des1,des2,k=2)
        # good = [[0,0]for i in range(len(matcher))]
        # for i, (m, n)in enumerate(matcher):
        #     if m.distance < 0.7*n.distance:
        #         good[i] = [1,0]
        good = []
        for m,n in matcher:
            if m.distance <0.7*n.distance:
                good.append(m)

        dst_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        src_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        camerMatrix = np.array([[fx,0,ppx],[0,fy,ppy],[0, 0, 1]],dtype=np.float32) 
        retval,rvec,tvec = cv2.solvePnP(dst_pts,src_pts,camerMatrix,None)
        print("rvec = ",rvec)
        print("tvec = ",tvec)
        
        # gray2=gray
        # kp2 = kp1
        des2 = des1
    
    cv2.imshow("image window",gray)

    


def get_distance():
    rospy.init_node('detect_harris_features',anonymous=True,disable_signals = False)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,callback)
    rospy.spin()

if __name__=='__main__':
    global fx,fy,ppx,ppy

    #K = [fx 0 ppx；0 fy ppy；0 0 1]
    fx = 609.68017578125
    fy = 608.42529296875
    ppx = 321.38134765625
    ppy = 238.43679809570

    get_distance()