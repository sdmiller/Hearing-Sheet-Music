import roslib
roslib.load_manifest("hsm_collection")
import cv2
import numpy as np

STANDARD_HEIGHT = 100
STANDARD_WIDTH = 300

def warp_measure(img, corners, desired_measure_height=STANDARD_HEIGHT, desired_measure_width=STANDARD_WIDTH):
    """ Assumes corners = (tl, bl, tr, br) of measure 
        Outputs img, staff line  y's
    """
    src = np.zeros((2,4) )
    for i,(cx,cy) in enumerate(corners):
        src[0,i] = cx
        src[1,i] = cy
    dst = np.array([ [0., 0, desired_measure_width, desired_measure_width],
                     [0, desired_measure_height, 0, desired_measure_height] ])
    H,mask = cv2.findHomography(src.transpose(), dst.transpose())
    img_warped = cv2.warpPerspective(img, H,(desired_measure_width,desired_measure_height))
    staff_lines = [n*desired_measure_height/8. for n in range(0,8)]
    return img_warped, staff_lines

def detect_corners(img):
    """ Given an image, output a list of "measures", where each measure is (tl, bl, tr, br)"""
    pass
