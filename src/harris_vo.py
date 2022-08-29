#!/usr/bin/python3
# encoding: utf-8


from sre_parse import State
import rospy
import cv2
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import String,Float32MultiArray,MultiArrayLayout,MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

state = 0
desc2 = []
filtered_coords2 = []

def callback(data):
    # pub = rospy.Publisher('/harris_vodom',Float32MultiArray,queue_size=2)
    global bridge,state,desc2,filtered_coords2
    bridge = CvBridge()
    
    rate = rospy.Rate(10)
    depth_image = bridge.imgmsg_to_cv2(data,'16UC1')
    depth_image = np.float32(depth_image)

    # method 1
    if state == 0 :
        filtered_coords2 = get_harris_points(depth_image)
        desc2 = get_descriptors(depth_image,filtered_coords2,5)
        print('1')
    else:
        
        filtered_coords1 = get_harris_points(depth_image)
        desc1 = get_descriptors(depth_image,filtered_coords1,5)
        matches = match_twosided(desc1,desc2,0.8)

        print('2')
        # print(matches)
        print(len(filtered_coords1))
        print(filtered_coords1,'\n')
        print(len(filtered_coords2))
        print(filtered_coords2)

        rvec, tvec = pnpsolve(filtered_coords1,filtered_coords2,matches)
        print("rvec = ",rvec)
        print("tvec = ",tvec)

        filtered_coords2 = filtered_coords1
        desc2 = desc1

    state = counter(state)
    rate.sleep()

    # method 2
    # filtered_coords1 = get_harris_points(depth_image)
    # desc1 = get_descriptors(depth_image,filtered_coords1,5)
    # filtered_coords2 = []
    # desc2 = []
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
        # print(desc)

    return desc


def match(desc1,desc2,threshold):
    n = len(desc1[0])
    d = -np.ones((len(desc1),len(desc2)))
    for i in range(len(desc1)):
        for j in range(len(desc2)):
            d1 = (desc1[i] - np.mean(desc1[i])) / np.std(desc1[i])
            d2 = (desc2[j] - np.mean(desc2[j])) / np.std(desc2[j])
            ncc_value = sum(d1*d2) / (n-1)
            if ncc_value > threshold:
                d[i, j] = ncc_value
    
    ndx = np.argsort(-d)
    matchscores = ndx[:, 0]

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
    frame1 = np.zeros((len(matches),2))
    frame2 = np.zeros((len(matches),2))
    print(len(matches))
    print(matches)
    index = 0
    for i, m in enumerate(matches):
        if m>0:
            frame1[index] = [coords1[i][0],coords1[i][1]]
            frame2[index] = [coords2[m][0],coords2[m][1]]
            index+=1
    # frame1 = np.delete(frame1,[index,len(matches)],axis=0)
    print(frame1)
    print(frame2)
    retval,rvec,tvec = cv2.solvePnP(frame1,frame2,camerMatrix,None)

    return rvec, tvec
    

def get_image():
    rospy.init_node('detect_harris_features',anonymous=True,disable_signals = False)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,callback)
    rospy.spin()


def counter(state):
    state+=1
    if(state>=100):
        state = 1

    return state


if __name__=='__main__':

    global fx,fy,ppx,ppy

    #K = [fx 0 ppx；0 fy ppy；0 0 1]
    fx = 609.68017578125
    fy = 608.42529296875
    ppx = 321.38134765625
    ppy = 238.43679809570

    get_image()