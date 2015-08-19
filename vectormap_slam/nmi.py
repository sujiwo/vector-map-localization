#!/usr/bin/python

from __future__ import division
import cv2
import numpy as np
import os
from sys import argv
import math
from copy import copy



def logp (val):
    try :
        return math.log(val)
    except :
        return 0


def histogramBinary1 (image1):
    hist = np.zeros(2)
    for r in range (image1.shape[0]) :
        for c in range (image1.shape[1]) :
            if image1[r,c] == 0 :
                hist[0] += 1
            else :
                hist[1] += 1
    return hist


def histogramBinary2 (image1, image2):
    hist = np.zeros ((2,2))
    if (image1.shape != image2.shape) :
        raise Exception ("Mismatched dimension")
    for r in range (image1.shape[0]) :
        for c in range (image1.shape[1]) :
            if image1[r,c] == 0 and image2[r,c] == 0:
                hist[0,0] += 1
            elif image1[r,c] !=0 and image2[r,c] == 0:
                hist[0,1] += 1
            elif image1[r,c] ==0 and image2[r,c] != 0:
                hist[1,0] += 1
            else :
                hist[1,1] += 1
    return hist


def imagebw ():
    im = np.zeros ((400,400), np.uint8)
    for i in range (im.shape[0]) :
        for j in range (im.shape[1]) :
            x = i/400.0 * math.pi * 8
            y = j/400.0
            im[i,j] = np.uint8(round((math.cos(x*y)+1)/2.)*255)
    return im


def entropy1 (image1):
    hst = histogramBinary1 (image1)
    N = image1.shape[0] * image1.shape[1]
    return sum ([p*logp(p) for p in hst/N]) * (-1)


def entropy2 (image1, image2):
    hst = histogramBinary2(image1, image2)
    N = image1.shape[0] * image1.shape[1]
    hst /= N
    plog = copy (hst)
    plog [0,0] = logp (plog[0,0])
    plog [0,1] = logp (plog[0,1])
    plog [1,0] = logp (plog[1,0])
    plog [1,1] = logp (plog[1,1])
    hst = hst * plog
    s = sum (sum(hst)) * (-1)
    return s


def nmi (image1, image2):
    return (entropy1(image1) + entropy1(image2)) / entropy2(image1, image2)


if __name__ == '__main__' :
    
    image1 = cv2.imread ("/var/tmp/test1.png", cv2.CV_LOAD_IMAGE_GRAYSCALE)
    image2 = cv2.imread ("/var/tmp/test2.png", cv2.CV_LOAD_IMAGE_GRAYSCALE)
    image3 = copy (image1)
    
    e1 = entropy2 (image1, image2)
    e2 = entropy2 (image1, image3)
    
    pass