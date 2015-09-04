from __future__ import division
import numpy as np
import math
import cv2

rectangle = np.array([
    [0, 0, -2],
    [1, 0, -2],
    [1, 1, -2],
    [0, 1, -2]
    ])


def normalize (vsrc):
    norm = np.linalg.norm(vsrc)
    if norm==0:
        return vsrc
    else:
        return vsrc / norm
    

def quaternion2matrix (_q):
    q = normalize (_q)
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    qmat = np.zeros((3,3))
    qmat[0,0] = 1 - 2*y**2 -2*z**2
    qmat[1,0] = 2*x*y + 2*w*z
    qmat[2,0] = 2*x*z - 2*w*y
    qmat[0,1] = 2*x*y - 2*w*z
    qmat[1,1] = 1 - 2*x**2 - 2*z**2
    qmat[2,1] = 2*y*z + 2*w*x
    qmat[0,2] = 2*x*z  + 2*w*y
    qmat[1,2] = 2*y*z - 2*w*x
    qmat[2,2] = 1 - 2*x**2 - 2*y**2
    return qmat


def perspective1 (fx, fy, cx, cy):
    projMat = np.array( \
        [[fx,  0,  cx,  0],
         [ 0, fy,  cy,  0],
         [ 0,  0,   1,  0]
         ])
    return projMat


def perspective2 (f, width, height):
    cx = width/2
    cy = height/2
    return perspective1 (f, f, cx, cy)


# Watch out on this: negative focal length !
def perspective3 (fieldOfView, width, height):
    fieldOfView = fieldOfView * math.pi / 180
    fieldOfView /= 2
    cx = width/2
    cy = height/2
    fx = width / (2*math.tan(fieldOfView))
    fy = fx
    return perspective1 (-fx, fy, cx, cy)

# def perspective4 (fieldOfView, aspectRatio, ):


def lookAt1 (position, orientation):
    position = np.array(position)
    viewMat = np.eye(4, 4)
    rotMat = quaternion2matrix(orientation)
    viewMat[0:3, 0:3] = rotMat
    viewMat[0:3, 3] = rotMat.dot (-position)
    return viewMat


def lookAt2 (_eyePosition, _pointOfView, _up) :
    eyePosition = np.array(_eyePosition)
    pointOfView = np.array(_pointOfView)
    up = normalize ( np.array(_up) )
    direction = normalize(pointOfView - eyePosition)
    # Fix the Up vector
    side = np.cross (direction, up)
    side = normalize(side)
    upt = np.cross (side, direction)
    # View matrix
    viewMat = np.eye(4,4)
    viewMat [0, 0:3] = side
    viewMat [1, 0:3] = upt
    viewMat [2, 0:3] = -direction
    viewMat [0:3, 3] = -eyePosition
    return viewMat

def project (point3d, viewMat, projMat):
    point4d = np.zeros(4)
    point4d[0:3] = point3d
    point4d[3] = 1
    p2 = projMat.dot(viewMat.dot(point4d))
    return p2 / p2[2]

def transform (point3d, viewMat):
    point4d = np.zeros(4)
    point4d[0:3] = point3d
    point4d[3] = 1
    p3 = viewMat.dot (point4d)
    return p3

# XXX: Unfinished !
def drawLineList (objectLines, image, viewMat, projMat):
    lineset = []
    for ip in range (len(objectLines)-1) :
        pass
        


if __name__ == '__main__' :
    viewmat = lookAt2 ([-0.5, -0.5, 2], [0.5, 0.5, -2], [0, 1, 0])
#     viewmat = lookAt2 ([-0.5, -0.5, 2], [0.5, 0.5, -2], [0, 1, 0])
#     viewmat = lookAt1((-0.915031, -0.943627, 1.656656), (0.985495, -0.117826, 0.121295, -0.014295))
    projmat = perspective3(45.0, 640, 480)
    
    image = np.zeros ((480,640), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    i = 0
    for point3d in rectangle :
        pcam = transform (point3d, viewmat)
        pprj = project (point3d, viewmat, projmat)
        point2 = pprj[0:2]
        (u,v) = int(point2[0]), int(point2[1])
        cv2.circle(image, (u,v), 3, 255)
        cv2.putText(image, str(i), (u,v), font, 1, 255)
        i += 1
    
    cv2.imwrite("/tmp/box.png", image)