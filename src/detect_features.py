#!/usr/bin/python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
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
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,3,3,0.04)
    dst = cv2.dilate(dst,None)

    harris = np.zeros((480,640),np.uint8)#blackground
    harris_points = dst>0.01*dst.max()
    harris[harris_points] = 200

    cv2.imshow("depth_image",depth_image)
    cv2.imshow("harris_features",harris)
    cv2.imshow("image window",gray)
    cv2.waitKey(3)

    # x = 360
    # y = 240
    # real_z = depth_image[x,y]
    # real_x = (x-ppx)/fx*real_z
    # real_y = (y-ppy)/fy*real_z

    


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