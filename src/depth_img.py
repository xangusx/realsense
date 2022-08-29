#!/usr/bin/python3
# encoding: utf-8

import rospy
from std_msgs.msg import String,Int64
import numpy as np
from sensor_msgs.msg import Image
import message_filters
import cv2
from cv_bridge import CvBridge,CvBridgeError
import pyrealsense2 as rs

def callback(data):
    global bridge
    bridge = CvBridge()
    pub = rospy.Publisher('/distance',Int64,queue_size=1)
    rate = rospy.Rate(10)
    depth_image = bridge.imgmsg_to_cv2(data,'16UC1')

    #define (x,y)
    x = 320
    y = 240

    real_z = depth_image[x,y]
    real_x = (x-ppx)/fx*real_z
    real_y = (y-ppy)/fy*real_z

    pub.publish(real_z)
    print(real_z*0.1,"cm")

    start_point = (x+10,y+10)
    end_point = (x-10,y-10)
    rect_color = 750
    dis_depth_img = cv2.rectangle(depth_image,start_point,end_point,rect_color,2)

    cv2.imshow("image window",dis_depth_img)
    cv2.waitKey(3)

    # colorizer = rs.colorizer()
    # colorizer_depth = np.asanyarray(colorizer.colorizer(depth_image).get_data())
    # cv2.imshow('colorizer_depth',colorizer_depth)

def get_distance():
    rospy.init_node('get_distance',anonymous=True,disable_signals = False)
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