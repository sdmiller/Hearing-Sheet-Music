#!/usr/bin/python

import roslib
roslib.load_manifest("hsm_collection")
import sys
import cv,cv2
import numpy as np

blockSize = 20
kernelSize = 3
k = 3

def main(args):
    im_path = args[0]
    global im
    im = cv2.imread(im_path,0)
    cv2.namedWindow("HARRIS")
    cv2.createTrackbar("blockSize","HARRIS",blockSize,100,set_blockSize)
    cv2.createTrackbar("kernelSize","HARRIS",kernelSize,31,set_kernelSize)
    cv2.createTrackbar("k","HARRIS",k,100,set_k)
    show_im()
    while True:
        cv2.waitKey(100)

def show_im():
    im_harris = cv2.cornerHarris(im, blockSize, kernelSize, k)
    im_harris = cv2.resize(im_harris,(im_harris.shape[1]/4,im_harris.shape[0]/4))
    cv2.imshow("HARRIS",abs(im_harris))


def set_blockSize(x):
    global blockSize
    blockSize = x
    show_im()

def set_kernelSize(x):
    global kernelSize
    kernelSize = x
    show_im()

def set_k(x):
    global k
    k = x
    show_im()

if __name__=='__main__':
    main(sys.argv[1:])
