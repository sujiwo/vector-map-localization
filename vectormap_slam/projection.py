from __future__ import division
import numpy as np
import math
import cv2

rectangle = []


def perspective1 (fx, fy, cx, cy):
    projMat = np.array( \
        [[fx,  0,  cx,  0],
         [ 0, fy,  cy,  0],
         [ 0,  0,   1,  0]
         ]
    )
    return projMat

def perspective2 (f, width, height):
    cx = width/2
    cy = height/2
    return perspective1 (f, f, cx, cy)

def perspective3 (fieldOfView, width, height):
    fieldOfView /= 2
    cx = width/2
    cy = height/2
    fx = width / (2*tan(fieldOfView))
    fy = fx/2
    return perspective1 (fx, fy, cx, cy)


if __name__ == '__main__' :
    print ("OK")
    pass