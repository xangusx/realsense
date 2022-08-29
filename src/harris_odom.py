#!/usr/bin/python3
# encoding: utf-8

import rospy
import cv2
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import String,Float32MultiArray,MultiArrayLayout,MultiArrayDimension,Int32
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

def callback(data):
    # pub = rospy.Publisher('/vodom',numpy_msg(Floats),queue_size=2)

    rate = rospy.Rate(10)
    harris_points[]
    harris_points = np.float32(data)


    # filtered_coords2 = get_harris_points(depth_image)
    # desc2 = get_descriptors(depth_image,filtered_coords2,5)

    # filtered_coords1 = get_harris_points(depth_image)
    # desc1 = get_descriptors(depth_image,filtered_coords1,5)
    # matches = match_twosided(desc1,desc2,0.8)
    # # print(matches)
    # rvec, tvec = pnpsolve(filtered_coords1,filtered_coords2,matches)
    # print("rvec = ",rvec)
    # print("tvec = ",tvec)
    # filtered_coords2 = filtered_coords1
    # desc2 = desc1
        


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


def get_descriptors(gray,filtered_coords,wid):  
    desc = []
    wid = 5
    for coords in filtered_coords:
        patch = gray[coords[0]-wid:coords[0]+wid+1,
                coords[1]-wid:coords[1]+wid+1].flatten()
        desc.append(patch)
        print(desc)

    return desc


def match(desc1,desc2,threshold):
    n = len(desc1[0])

    d = -np.ones((len(desc1,len(desc2))))
    for i in range(len(desc1)):
        for j in range(len(desc2)):
            d1 = (desc1[i] - np.mean(desc1[i])) / np.std(desc1[i])
            d2 = (desc2[j] - np.mean(desc2[j])) / np.std(desc2[j])
            ncc_value = sum(d1*d2) / (n-1)
            if ncc_value > threshold:
                d[i, j] = ncc_value
    
    ndx = np.argsort(-d)
    matchscores = ndx[:,0]

    return matchscores


def match_twosided(desc1,desc2,threshold):
    matches_12 = match(desc1,desc2,threshold)
    matches_21 = match(desc2,desc1,threshold)
    ndx_12 = np.where(matches_12 >=0)[0]

    for n in ndx_12:
        if matches_21[matches_12[n]] != n:
            matches_12[n] = -1

    return matches_12


def pnpsolve(coords1,coords2,matches):
    camerMatrix = np.array([[fx,0,ppx],[0,fy,ppy],[0, 0, 1]],dtype=np.float32)
    retval,rvec,tvec = cv2.solvePnP(coords1,coords2,camerMatrix,None)
    return rvec, tvec
    # for i, m in enumerate(matches):
    #     if m>0:
    #         coords1[i][1],coords2[m][1]
    #         coords1[i][0],coords2[m][0]

def get_harris_points():
    rospy.init_node('vodom',anonymous=True,disable_signals = False)
    rospy.Subscriber('/harris_points',numpy_msg(Floats),callback)
    rospy.spin()


if __name__=='__main__':

    global fx,fy,ppx,ppy

    #K = [fx 0 ppx；0 fy ppy；0 0 1]
    fx = 609.68017578125
    fy = 608.42529296875
    ppx = 321.38134765625
    ppy = 238.43679809570

    get_harris_points()