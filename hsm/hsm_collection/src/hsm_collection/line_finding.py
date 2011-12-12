#!/usr/bin/python
import roslib
roslib.load_manifest("hsm_collection")
import numpy as np
import cv2
(HOUGH,HOUGH_PROB) = range(2)

def get_corners(img,mode=HOUGH):
    """ get_corners(img) -> corner_mat
    Looks for roughly 90 degree corners in an
    image.
    If mode == HOUGH, searches for full lines
    If mode == HOUGH_PROB, searches for line segments
    """
    pass

# Some useful utility functions for working with lines

def draw_hough_lines(im,lines):
    """ draw_hough_lines(im,lines) -> [vert_im, horiz_im] """
    vert_im = np.zeros(im.shape[:2],'uint8')
    horiz_im = np.zeros(im.shape[:2],'uint8')
    for b,theta in lines[0]:
        #line perp = -b*sin(theta),b*cos(theta)ctr
        #ctr = b*cos(theta),b*sin(theta)
        theta = theta - np.pi/2.
        ctr_offset = np.array((-b*np.sin(theta),b*np.cos(theta)))
        
        ln = np.array((b*np.cos(theta),b*np.sin(theta)))
        #start_pt = tuple(-1000*ln+ctr_offset)
        #end_pt = tuple(1000*ln+ctr_offset)
        pts = [vert_inter(ln,ctr_offset,i) for i in (0,im.shape[1])]
        pts.extend([horiz_inter(ln,ctr_offset,i) for i in (0,im.shape[0])])
        pts = [pt for pt in pts if legal_pt(pt,im.shape)]
        #pts = sorted(pts,key = lambda pt: 0-pt[0] + 0-pt[1] + pt[0]-im.shape[1] + pt[1]-im.shape[0] )
        start_pt = pts[0]
        end_pt = pts[1]
        #end_pt = (im.shape[1],im.shape[1]*np.tan(theta)+b)
        if np.abs( (theta) ) < np.pi/6:
            cv2.line(horiz_im,start_pt,end_pt,255,8)
        elif np.abs( (theta) ) > (np.pi/2-np.pi/6):
            cv2.line(vert_im,start_pt,end_pt,255,8)
        #cv2.line(im,start_pt,end_pt,(0,0,255),2)
    return vert_im,horiz_im

def legal_pt(pt,shape):
    eps = 1
    return pt[0]+eps > 0 and shape[1] - pt[0] +eps > 0 and pt[1]+eps > 0 and shape[0] - pt[1] +eps > 0

def horiz_inter(ln,offset,inter):
    if ln[1] == 0:
        return (np.inf,np.inf)
    else:
        return (int((inter - offset[1])/ln[1]*ln[0]+offset[0]),int(inter))

def vert_inter(ln,offset,inter):
    if ln[0] == 0:
        return (np.inf,np.inf)
    else:
        return (int(inter),int((inter - offset[0])/ln[0]*ln[1]+offset[1]))
