#!/usr/bin/env python
#Find objects
#Find objects with color green
#Find their position, size and orientation
#Calibration

import numpy as np
import cv2


def find_objects(initimg,img):
    return cv2.absdiff(img,initimg)


if __name__ == '__main__':
    img = cv2.imread('/home/magnaars/catkin_ws/src/five_dof_robotarm/img/topview.png',cv2.IMREAD_UNCHANGED)
    initimg = cv2.imread('/home/magnaars/catkin_ws/src/five_dof_robotarm/img/topview_empty.png',cv2.IMREAD_UNCHANGED)
    objects = find_objects(initimg,img)


    cv2.imshow('image',objects)
    cv2.waitKey(20000) #milliseconds
    cv2.destroyAllWindows()
