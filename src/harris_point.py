#!/usr/bin/python3
# encoding: utf-8


from sre_parse import State
import rospy
import cv2
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import String,Float32MultiArray,MultiArrayLayout,MultiArrayDimension,Int32
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


def callback(data):
    pub = rospy.Publisher('/harris_points',numpy_msg(Floats),queue_size=2)
    global bridge
    bridge = CvBridge()
    
    rate = rospy.Rate(10)
    depth_image = bridge.imgmsg_to_cv2(data,'16UC1')
    depth_image = np.float32(depth_image)
    filtered_coords = get_harris_points(depth_image)
    # print(filtered_coords)
    # print(type(filtered_coords))

    harris_coords = np.array(filtered_coords, dtype=np.float32)
    # print(harris_coords)
    # print(type(harris_coords))
    pub.publish(harris_coords)



def get_harris_points(gray):
    dst = cv2.cornerHarris(gray,3,3,0.04)
    dst = cv2.dilate(dst,None)

    harris_points = dst>0.01*dst.max()
    coords = np.array(harris_points.nonzero()).T
    candidate_values = [dst[c[0],c[1]]for c in coords]
    index = np.argsort(candidate_values)[::-1]

    allow_locations = np.zeros(dst.shape)
    min_dist = 10
    allow_locations[min_dist:-min_dist,min_dist:-min_dist] = 1

    filtered_coords = []
    for i in index :
        if allow_locations[coords[i,0],coords[i,1]] == 1:
            filtered_coords.append(coords[i])
            allow_locations[(coords[i,0]-min_dist):(coords[i,0]+min_dist),
                            (coords[i,1]-min_dist):(coords[i,1]+min_dist)] = 0

    return filtered_coords

def get_image():
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

    get_image()