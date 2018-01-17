#!/usr/bin/env python
#Find objects
#Find objects with color green

import numpy as np
import cv2

# Load an color image in grayscale


if __name__ == '__main__':
    img = cv2.imread('/home/magnaars/catkin_ws/src/five_dof_robotarm/img/topview.png',0)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
