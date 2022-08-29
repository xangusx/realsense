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
    blur = cv2.GaussianBlur(gray,(7,7),5)
    blur = np.uint8(blur)
    edges = cv2.Canny(blur,10,30)

    line_data = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50, minLineLength=300)

    # background = np.zeros((480,640),np.uint8)
    background = np.copy(gray)*0
    if line_data is not None :
        # print('line:',len(line_data))
        for lines in line_data :
            x1, y1, x2, y2 = lines[0]
            cv2.line(background,(x1,y1),(x2,y2),100,2)
            # print((x2-x1)/(y2-y1))
    # else :
        # print('nothing')

    # cv2.imshow("depth_image",depth_image)
    cv2.imshow("gray",gray)
    # cv2.imshow("blur",blur)
    cv2.imshow("edge",edges)
    cv2.imshow("line",background)
    cv2.waitKey(3)



def get_distance():
    rospy.init_node('detect_harris_features',anonymous=True,disable_signals = False)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,callback)
    rospy.spin()

if __name__=='__main__':
    global fx,fy,ppx,ppy
    global slope

    #K = [fx 0 ppx；0 fy ppy；0 0 1]
    fx = 609.68017578125
    fy = 608.42529296875
    ppx = 321.38134765625
    ppy = 238.43679809570

    get_distance()