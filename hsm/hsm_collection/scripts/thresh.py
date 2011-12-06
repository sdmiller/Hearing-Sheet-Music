#!/usr/bin/python

import roslib
roslib.load_manifest("hsm_collection")
import sys
import cv,cv2
import numpy as np

h_min=0
h_max=360/2
v_min=0
v_max=256
s_min=0
s_max=256

bwthresh=125
filtsize=50

dog_k=7
dog_sig1=4
dog_sig2=15

canny_thresh1=0
canny_thresh2=0


rho = 10
theta_disc = 10
hough_thresh = 100
minlinesize=50
maxgap = 2

def main(args):
    im_path = args[0]
    global im
    im = cv2.imread(im_path)
    global im_hsv
    im_hsv = cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
    cv2.namedWindow("THRESH")
    cv2.namedWindow("MF")
    cv2.namedWindow("DoG")
    #cv2.createTrackbar("Hue Min","THRESH",0,360/2,set_hue_min)
    #cv2.createTrackbar("Hue Max","THRESH",360/2,360/2,set_hue_max)
    #cv2.createTrackbar("Sat Min","THRESH",0,256,set_sat_min)
    #cv2.createTrackbar("Sat Max","THRESH",256,256,set_sat_max)
    #cv2.createTrackbar("Val Min","THRESH",0,256,set_val_min)
    #cv2.createTrackbar("Val Max","THRESH",256,256,set_val_max)
    cv2.createTrackbar("Thresh","THRESH",bwthresh,256,set_bwthresh)
    cv2.createTrackbar("Box Filter Size","THRESH",filtsize,500,set_filtsize)
    
    cv2.createTrackbar("Kernel","DoG",dog_k,51,set_dog_k)
    cv2.createTrackbar("Sig1/10","DoG",dog_sig1,100,set_dog_sig1)
    cv2.createTrackbar("Sig2/10","DoG",dog_sig2,100,set_dog_sig2)
    cv2.namedWindow("HOUGH")
    cv2.createTrackbar("rho/10","HOUGH",rho,100,set_rho)
    cv2.createTrackbar("theta_disc","HOUGH",theta_disc,180,set_theta_disc)
    cv2.createTrackbar("hough_thresh","HOUGH",hough_thresh,1000,set_hough_thresh)
    cv2.createTrackbar("minlinesize","HOUGH",minlinesize,1000,set_minlinesize)
    cv2.createTrackbar("maxgap","HOUGH",maxgap,1000,set_maxgap)
    cv2.namedWindow("CANNY")
    cv2.createTrackbar("canny_thresh1","CANNY",canny_thresh1,255,set_canny_thresh1)
    cv2.createTrackbar("canny_thresh2","CANNY",canny_thresh2,255,set_canny_thresh2)

    show_im()
    while True:
        cv2.waitKey(100)

def show_im():
    #(h,s,v) = cv2.split(im_hsv)
    #h_mask = min_max_thresh(h,h_min,h_max)
    #s_mask = min_max_thresh(s,s_min,s_max)
    #v_mask = min_max_thresh(v,v_min,v_max)
    #tot_mask = cv2.bitwise_and(cv2.bitwise_and(h_mask,s_mask),v_mask)
    #masked_im = cv2.bitwise_and(im,cv2.merge([tot_mask,tot_mask,tot_mask]))
    #cv2.imshow("THRESH",masked_im)

    #dx = cv2.Sobel(h,8,1,0)
    #dy = cv2.Soble(h,8,0,1)
    #cv2.imshow("LAP",dx^2+dy^2)
    imgray = cv2.cvtColor( im,cv2.COLOR_BGR2GRAY)
    meanfiltered = imgray - cv2.boxFilter(imgray,8,(filtsize,filtsize))+128
    meanfiltered = cv2.blur(meanfiltered,(3,1))
    #cv2.imshow("THRESH",meanfiltered)
    gauss1 = cv2.GaussianBlur(imgray,(dog_k,dog_k),dog_sig1/10.)
    gauss2 = cv2.GaussianBlur(imgray,(dog_k,dog_k),dog_sig2/10.)
    DoG = gauss1-gauss2
    cv2.imshow("DoG",DoG)
    canny_edges = cv2.Canny(imgray,canny_thresh1,canny_thresh2);
    cv2.imshow("CANNY",canny_edges)
    lines = np.array([])
    #HOUGH_IM=canny_edges
    r,imbw = cv2.threshold(DoG,bwthresh,255,cv2.THRESH_BINARY)
    cv2.imshow("THRESH",imbw)
    HOUGH_IM=imbw
    lines = cv2.HoughLinesP(HOUGH_IM,rho/10.,theta_disc*np.pi/180.,hough_thresh,lines,minlinesize,maxgap)
    maxlen = np.sqrt(im.shape[0]**2+im.shape[1]**2)
    imcp = im.copy()
    for ln in lines[0]:
        pt1 = (ln[0],ln[1])
        pt2 = (ln[2],ln[3])
        cv2.line(imcp,pt1,pt2,(0,0,255))
    cv2.imshow("HOUGH",imcp)


def min_max_thresh(im,lb,ub):
    r,l = cv2.threshold(im,lb,255,cv2.THRESH_BINARY)
    r,u = cv2.threshold(im,ub,255,cv2.THRESH_BINARY_INV)
    return cv2.bitwise_and(l,u)

def set_rho(x):
    global rho
    rho = x
    show_im()

def set_theta_disc(x):
    global theta_disc
    theta_disc = x
    show_im()

def set_hough_thresh(x):
    global hough_thresh
    hough_thresh = x
    show_im()

def set_minlinesize(x):
    global minlinesize
    minlinesize = x
    show_im()

def set_maxgap(x):
    global maxgap
    maxgap = x
    show_im()

def set_bwthresh(x):
    global bwthresh
    bwthresh = x
    show_im()

def set_filtsize(x):
    global filtsize
    filtsize = x
    show_im()

def set_dog_k(x):
    global dog_k
    dog_k = x
    show_im()

def set_dog_sig1(x):
    global dog_sig1
    dog_sig1 = x
    show_im()

def set_dog_sig2(x):
    global dog_sig2
    dog_sig2 = x
    show_im()

def set_hue_min(x):
    global h_min
    h_min = x
    show_im()

def set_hue_max(x):
    global h_max
    h_max = x
    show_im()

def set_sat_min(x):
    global s_min
    s_min = x
    show_im()

def set_sat_max(x):
    global s_max
    s_max = x
    show_im()

def set_val_min(x):
    global v_min
    v_min = x
    show_im()

def set_val_max(x):
    global v_max
    v_max = x
    show_im()

def set_canny_thresh1(x):
    global canny_thresh1
    canny_thresh1 = x
    show_im()

def set_canny_thresh2(x):
    global canny_thresh2
    canny_thresh2 = x
    show_im()

if __name__=='__main__':
    main(sys.argv[1:])
