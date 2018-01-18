#!/usr/bin/env python
#Find objects
#Find objects with color green
#Find their position, size and orientation
#Calibration

import numpy as np
import cv2
import math

def distance(p1,p2):
     return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def find_point_nearest_center(intersectionpoints,img):
    imgshape = img.shape
    center = [imgshape[0]/2,imgshape[1]/2]

    nearest_point = intersectionpoints[0]
    lowest_dist = distance(nearest_point,center)
    for point in intersectionpoints:
        dist = distance(point,center)
        if dist < lowest_dist:
            nearest_point = point
            lowest_dist = dist
    return nearest_point

def find_point_nearest_point(intersectionpoints,point):

    nearest_point = intersectionpoints[0]
    lowest_dist = distance(nearest_point,point)

    for point in intersectionpoints:
        dist = distance(point,point)
        if dist < lowest_dist:
            nearest_point = point
            lowest_dist = dist
    return nearest_point


def subtractlists(list1,list2):
    endlist = range(len(list1))
    i = 0
    for item in list1:
        endlist[i] = item-list2[i]
        i = i + 1
    return endlist

def find_intersection( p0, p1, p2, p3 ) :
    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]

    denom = s10_x * s32_y - s32_x * s10_y

    if denom == 0 : return None # collinear

    denom_is_positive = denom > 0

    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]

    s_numer = s10_x * s02_y - s10_y * s02_x

    if (s_numer < 0) == denom_is_positive : return None # no collision

    t_numer = s32_x * s02_y - s32_y * s02_x

    if (t_numer < 0) == denom_is_positive : return None # no collision

    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision


    # collision detected

    t = t_numer / denom

    intersection_point = [ p0[0] + (t * s10_x), p0[1] + (t * s10_y) ]


    return intersection_point

def find_diff(initimg,img):
    return cv2.absdiff(img,initimg)

def find_corner_coord(dst,img):
    tshape = dst.shape
    circ = img
    #circ[:] = 0
     # or img[:] = 255
    i = 0
    for x in range(0,tshape[0]):
        for y in  range(0,tshape[1]):
            if dst[x,y]>0.01:
                circ =  cv2.circle(circ,(y,x), 3, (0,0,255), 0)
                i = i + 1
    print i
    return img

def hough_lines(img):

    imgshape =  img.shape
    val = max(imgshape[0],imgshape[1])

    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)

    lines = cv2.HoughLines(edges,1,np.pi/180,200)

    interseclines = []

    for line in lines:
        for rho,theta in line:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + val*(-b))
            y1 = int(y0 + val*(a))
            x2 = int(x0 - val*(-b))
            y2 = int(y0 - val*(a))

            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
            interseclines.append([[x1+0.0,y1+0.0],[x2+0.0,y2+0.0]])

    firsthalf = interseclines[:len(interseclines)/2]
    secondhalf = interseclines[len(interseclines)/2:]

    intersectionpoints = []
    for line1 in interseclines:
        for line2 in interseclines:
            intersecp = find_intersection(line1[0], line1[1], line2[0], line2[1])
            if intersecp != None and intersecp not in intersectionpoints:
                intersectionpoints.append(intersecp)
                cv2.circle(img,(int(intersecp[0]),int(intersecp[1])), 8, (0,150,0), 4)

    #return img #if one want to return the img with lines plotted
    return intersectionpoints

def harris_corner(img):
    #https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html
    gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    gray = np.float32(gray)

    dst = cv2.cornerHarris(gray,2,3,0.04)
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    img[dst>0.01*dst.max()]=[0,255,0,0]
    #print np.unravel_index(dst.argmax(),img.shape)
    return img,dst


if __name__ == '__main__':
    img = cv2.imread('/home/magnaars/catkin_ws/src/five_dof_robotarm/img/topview.png',cv2.IMREAD_UNCHANGED)
    initimg = cv2.imread('/home/magnaars/catkin_ws/src/five_dof_robotarm/img/topview_empty.png',cv2.IMREAD_UNCHANGED)
    chessboard = cv2.imread('/home/magnaars/catkin_ws/src/five_dof_robotarm/img/onlychess.png',cv2.IMREAD_UNCHANGED)

    #cornersimg,corners = harris_corner(chessboard)
    #cirimg = find_corner_coord(corners,chessboard)
    intersectionpoints = hough_lines(chessboard)
    c_point = find_point_nearest_center(intersectionpoints,img)
    n_point = find_point_nearest_point(intersectionpoints,c_point)

    #Number of pixels of one square
    n_pixels = distance(c_point,n_point)

    #Length of one square in meter
    l_square = 0.25

    #Length of one pixel in meter
    one_pixel = l_square/n_pixels

    #Length of one meter in pixels
    one_meter = 1/one_pixel









    #line_intersection(0,0,2,2,0,2,2,0)

    #cv2.imshow('image',intersectionpoints)
    #cv2.waitKey(5000) #milliseconds
    cv2.destroyAllWindows()
