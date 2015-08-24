from __future__ import division
import cv2
import numpy as np
import math


points = np.array([[0, 0, -2],
    [1, 0, -2],
    [1, 1, -2],
    [0, 1, -2]])


def degreeToRadian (degree) :
    return degree * np.pi / 180.0;


def lookAt (eyePos, centerOfView, upVect):
    viewMat = np.eye (4)
    direction = centerOfView - eyePos
    direction = direction / np.linalg.norm (direction)
    
    upVect = upVect / np.linalg.norm (upVect)
    side = np.cross (direction, upVect)
    Utrue = np.cross (side, direction)
    
    viewMat [0, 0:3] = side
    viewMat [1, 0:3] = Utrue
    viewMat [2, 0:3] = -direction
    viewMat [0:3, 3] = -eyePos
    
    return viewMat

def perspective (fovy, aspectRatio, zNear, zFar):
#     projectionMatrix = Matrix4::Zero();
#     double angle = degreeToRadian(fovy/2);
#     double f = 1 / (tan(angle));
#     projectionMatrix (0, 0) = f/aspectRatio;
#     projectionMatrix (1, 1) = f;
#     projectionMatrix (2, 2) = (zFar + zNear) / (zNear - zFar);
#     projectionMatrix (3, 2) = -1;
#     projectionMatrix (2, 3) = 2 * zFar * zNear / (zNear - zFar);
    projMat = np.eye (4)
    angle = degreeToRadian(fovy/2.0)
    f = 1 / math.tan (angle)
    projMat [0, 0] = f / aspectRatio
    projMat [1, 1] = f
    projMat [2, 2] = (zFar + zNear) / (zNear - zFar)
    projMat [3, 2] = -1
    projMat [2, 3] = 2 * zFar * zNear / (zNear - zFar)
    return projMat  



if __name__ == '__main__' :
    viewMat = lookAt (np.array([-0.5, -0.5, 2]),
        np.array([0.5, 0.5, -2]),
        np.array([0, 1, 0]))
    projMat = perspective(45.0, 1.33, 1.0, 100)
    
    image = np.zeros ((1024,748,3), dtype=np.uint8)
    rv, jac = cv2.Rodrigues(viewMat[0:3, 0:3])
    imgpts, jac = cv2.projectPoints(points, rv, viewMat[0:3, 3], projMat, 0)
    
    pass