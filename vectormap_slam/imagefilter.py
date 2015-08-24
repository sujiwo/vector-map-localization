#!/usr/bin/python

from __future__ import division
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from sys import argv


imageTopic = "/camera1"
imageSize = (1024, 768)
mask = None
detector = cv2.ORB(200)


def DetectFeaturesFromLines (imagesrc):
    global mask, detector
    
    imageproc = cv2.GaussianBlur(imagesrc, (5,5), 0)
#     imageproc = cv2.Sobel(imageproc, -1, 1, 1, imageproc, 3, 1, 0.5)
#     edges = cv2.Canny(imageproc, 50, 200, apertureSize=5)

    imageproc = cv2.Laplacian(imageproc, -1)
    
    keypoints, descriptors = detector.detectAndCompute (imageproc, None)
    imageproc = cv2.cvtColor(imageproc, cv2.COLOR_GRAY2BGR)
    imageproc = cv2.drawKeypoints(imageproc, keypoints, flags=2, color=[0, 255, 0])

    return imageproc


def DetectLines (imagesrc):
    
    imageproc = cv2.GaussianBlur(imagesrc, (11,11), 0)
    edges = cv2.Canny(imageproc, 1, 50)
    
    lines = cv2.HoughLinesP(edges, 100, np.pi/180.0, 300, 100)
    imageproc = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    print ("# of lines: {}".format(len(lines[0])))

    for l in lines[0]:
        cv2.line (imageproc, (l[0],l[1]), (l[2],l[3]), (0,0,255), 2)
        pass
    
    return imageproc
    


def imageHandler (imageMsg):
    global imageWindow, bridge, isSaved, mask, imageSize
    global detector
    
    image = bridge.imgmsg_to_cv2(imageMsg, 'mono8')
    image = cv2.resize(image, imageSize)
    
    # Apply mask
    cv2.bitwise_and(image, mask, image)
    imageproc = DetectLines (image)    
    
    cv2.imshow("ImageProcessor", imageproc)
    cv2.imshow("CameraImage", image)
    
    
def glHandler (imageMsg):
    global mask, bridge, imageSize
    
    image = bridge.imgmsg_to_cv2 (imageMsg, 'rgba8')
    #image = cv2.resize (image, imageSize)
    cv2.imshow("MapProjection", image)


if (__name__ == '__main__') :
    
    cv2.startWindowThread()
    imageWindowProc = cv2.namedWindow("ImageProcessor", cv2.WINDOW_AUTOSIZE)
    imageWindowOrig = cv2.namedWindow("CameraImage", cv2.WINDOW_AUTOSIZE)
    imageWindowGl = cv2.namedWindow ("MapProjection", cv2.WINDOW_AUTOSIZE)
    bridge = CvBridge()
    
    # Read mask
    d = os.path.dirname (argv[0])
    mask = cv2.imread(os.path.dirname(argv[0]) + "/mask.png")
    mask = cv2.resize(mask, imageSize)
    mask = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
    
    rospy.init_node ("ImageTest", anonymous=True)
    rospy.Subscriber (imageTopic + "/image_raw", Image, imageHandler)
    rospy.Subscriber ("/glimage", Image, glHandler)
    rospy.spin()
    pass
