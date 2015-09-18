from __future__ import division
import numpy as np
import math
from math import sin, cos, tan
import cv2

# rectangle = np.array([
#     [0, 0, -2],
#     [1, 0, -2],
#     [1, 1, -2],
#     [0, 1, -2]
#     ])
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


def createPose (_eyePosition, _pointOfView, _up):
    viewMat = lookAt2 (_eyePosition, _pointOfView, _up)
    return createPoseFromViewMatrix(viewMat)

    
def createPoseFromViewMatrix (viewMat):
    M = viewMat[0:3, 0:3]
    trace = M.trace()
    if trace > 0:
        S = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * S
        qx = (M[2,1] - M[1,2]) / S
        qy = (M[0,2] - M[2,0]) / S
        qz = (M[1,0] - M[0,1]) / S
    elif ((M[0,0] > M[1,1]) and (M[0,0] > M[2,2])) :
        S = math.sqrt (1.0 + M[0,0] - M[1,1] - M[2,2]) * 2
        qw = (M[2,1] - M[1,2]) / S
        qx = 0.25 * S
        qy = (M[0,1] + M[1,0]) / S
        qz = (M[0,2] + M[2,0]) / S
    elif (M[1,1] > M[2,2]) :
        S = math.sqrt (1.0 + M[1,1] - M[0,0] - M[2,2]) * 2
        qw = (M[0,2] - M[2,0]) / S
        qx = (M[0,1] + M[1,0]) / S
        qy = 0.25 * S
        qz = (M[1,2] + M[2,1]) / S
    else :
        S = math.sqrt (1.0 + M[2,2] - M[0,0] - M[1,1]) * 2
        qw = (M[1,0] - M[0,1]) / S;
        qx = (M[0,2] + M[2,0]) / S;
        qy = (M[1,2] + M[2,1]) / S;
        qz = 0.25 * S
    orientation = (qw, qx, qy, qz)
    position = -M.transpose() .dot( viewMat[0:3, 3] )
    return position, orientation


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


# This function returns OpenGL-style projection matrix
# ie. Normalized Device Coordinate
def perspective4 (fieldOfView, aspectRatio, near=0.1, far=100.0):
    projMat = np.zeros((4,4))
    
    fieldOfView = fieldOfView * math.pi / 180
    d = 1 / math.tan(fieldOfView/2)

    projMat[0,0] = d/aspectRatio
    projMat[1,1] = d
    projMat[2,2] = (near+far)/(near-far)
    projMat[2,3] = 2*near*far/(near-far)
    projMat[3,2] = -1
    
    return projMat


def perspective5 (fx, fy, cx, cy, width, height, near=0.1, far=100.0):
    projMat = np.zeros((4,4))
    
    projMat [0,0] = 2.0*fx/width
    projMat [1,1] = 2.0*fy/height
    projMat [0,2] = 2.0*(0.5 - cx/width)
    projMat [1,2] = 2.0*(cy/height - 0.5)
    projMat [2,2] = -(far+near) / (far-near)
    projMat [2,3] = -2*.0*far*near / (far-near)
    projMat [3,2] = -1
    
    return projMat


def perspective6 (fieldOfView, width, height):
    fieldOfView = fieldOfView * math.pi / 180
    fieldOfView /= 2
    cx = width/2
    cy = height/2
    fx = width / (2*math.tan(fieldOfView))
    fy = fx
    return perspective5 (fx, fy, cx, cy, width, height)


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


# Look stupid? Yeah, I love verbosity
def lookAt3 (eyePosition, roll, pitch, yaw):
    
    # convert from degree to radian
    roll = roll * math.pi / 180.0
    pitch = pitch * math.pi / 180.0
    yaw = yaw * math.pi / 180.0
    
    viewMat = np.eye (4,4)
    
    mroll = np.eye(3, 3)
    mroll[0,0] = cos (roll)
    mroll[0,1] = -sin (roll)
    mroll[1,0] = sin (roll)
    mroll[1,1] = cos (roll)
    
    mpitch = np.eye(3,3)
    mpitch[1,1] = cos (pitch)
    mpitch[1,2] = -sin (pitch)
    mpitch[2,1] = sin (pitch)
    mpitch[2,2] = cos (pitch)   

    myaw = np.eye(3,3)
    myaw[0,0] = cos (yaw)
    myaw[0,2] = sin (yaw)
    myaw[2,0] = -sin (yaw)
    myaw[2,2] = cos (yaw)    
    
    # Test first !
    rotmat = myaw.dot(mpitch.dot(mroll))
    viewMat[0:3,3] = rotmat.dot (-np.array(eyePosition))
    viewMat[0:3,0:3] = rotmat
    
    return viewMat


def project1 (point3d, viewMat, projMat):
    point4d = np.zeros(4)
    point4d[0:3] = point3d
    point4d[3] = 1
    p2 = projMat.dot(viewMat.dot(point4d))
    return p2 / p2[2]


def project2 (point3d, viewMat, projMat, width=0, height=0):
    point4d = np.zeros(4)
    point4d[0:3] = point3d
    point4d[3] = 1
    ps = projMat.dot(viewMat.dot(point4d))
    ps = ps / ps[3]
    p2 = ps[0:2] / ps[2]
    if (width==0 and height==0) :
        return p2
    else :
        # Convert normalized device coordinate to screen coordinate
        p2[0] = width * (1+p2[0]) / 2
        p2[1] = height * (1-p2[1]) / 2
        return p2
        

def transform (point3d, viewMat):
    point4d = np.zeros(4)
    point4d[0:3] = point3d
    point4d[3] = 1
    p3 = viewMat.dot (point4d)
    return p3

# XXX: Unfinished !
def drawLineList1 (objectLines, image, viewMat, projMat):
    
    def drawLine (p1, p2):
        p1p = project1 (p1, viewMat, projMat)
        p2p = project1 (p2, viewMat, projMat)
        (u1, v1) = int(p1p[0]), int(p1p[1])
        (u2, v2) = int(p2p[0]), int(p2p[1])
#         cv2.circle (image, (u1, v1), 2, 255)
#         cv2.circle (image, (u2, v2), 2, 255)
        cv2.line (image, (u1,v1), (u2,v2), 255)
    
    for ip in range (len(objectLines)-1) :
        drawLine (objectLines[ip], objectLines[ip+1])
    drawLine (objectLines[len(objectLines)-1], objectLines[0])


def drawLineList2 (objectLines, image, viewMat, projMat):
    
    def drawLine (p1, p2):
        p1p = project2 (p1, viewMat, projMat, image.shape[1], image.shape[0])
        p2p = project2 (p2, viewMat, projMat, image.shape[1], image.shape[0])
        (u1, v1) = int(p1p[0]), int(p1p[1])
        (u2, v2) = int(p2p[0]), int(p2p[1])
#         cv2.circle (image, (u1, v1), 2, 255)
#         cv2.circle (image, (u2, v2), 2, 255)
        cv2.line (image, (u1,v1), (u2,v2), 255)
    
    for ip in range (len(objectLines)-1) :
        drawLine (objectLines[ip], objectLines[ip+1])
    drawLine (objectLines[len(objectLines)-1], objectLines[0])

    
    
def drawPoints1 (object, image, viewMat, projMat):
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    i = 0
    for point3d in object :
        pcam = transform (point3d, viewmat)
        pprj = project1 (point3d, viewmat, projmat)
        point2 = pprj[0:2]
        (u,v) = int(point2[0]), int(point2[1])
        cv2.circle(image, (u,v), 3, 255)
        cv2.putText(image, str(i), (u,v), font, 1, 255)
        i += 1


def drawPoints2 (object, image, viewMat, projMat):
    font = cv2.FONT_HERSHEY_SIMPLEX
    width = image.shape[1]
    height = image.shape[0]
    
    i = 0
    for point3d in object :
        pcam = transform (point3d, viewmat)
        point2 = project2 (point3d, viewmat, projmat, width, height)
        (u,v) = int (point2[0]) , int (point2[1])
        cv2.circle(image, (u,v), 3, 255)
        cv2.putText(image, str(i), (u,v), font, 1, 255)
        i += 1




if __name__ == '__main__' :
#     viewmat = lookAt2 ([-0.5, -0.5, 2], [0.5, 0.5, -2], [0, 1, 0])
#     position, orientation = createPoseFromViewMatrix(viewmat)
#     viewmat = lookAt2 ([0.5, 0.5, 2], [0.5, 0.5, -2], [0, 1, 0])
    viewmat = lookAt3 ([-0.5, -0.5, 2], 0, 10, -10)
#     viewmat = lookAt1((-0.915031, -0.943627, 1.656656), (0.985495, -0.117826, 0.121295, -0.014295))
    projmat = perspective6(75.0, 640, 480)
#     projmat = perspective4 (45.0, 1.333)
    image = np.zeros ((480,640), dtype=np.uint8)
#     drawPoints2 (rectangle, image, viewmat, projmat)
    drawLineList2 (rectangle, image, viewmat, projmat)
    cv2.imwrite ("/tmp/box.png", image)
    
    pass