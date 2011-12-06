import roslib
roslib.load_manifest("hsm_collection")
import sys
import cv2
import numpy as np

class BoundingBox:
    def __init__(self,origin,width,height):
        self.origin = origin
        self.width = width
        self.height = height

    @classmethod
    def from_pts(cls,pt1,pt2):
        xmin = min(pt1[0],pt2[0])
        xmax = max(pt1[0],pt2[0])
        ymin = min(pt1[1],pt2[1])
        ymax = max(pt1[1],pt2[1])
        origin = (xmin,ymin)
        width = xmax-xmin
        height = ymax-ymin
        return BoundingBox(origin,width,height)

    def draw_to_image(self,im,color=(0,0,0),thickness=2):
        cv2.rectangle(im,self.origin
                     ,(self.origin[0]+self.width,self.origin[1]+self.height)
                     ,color, thickness )

    def shift(self,dx,dy):
        self.origin = (self.origin[0]+dx,self.origin[1]+dy)

class LabeledExample:
    def __init__(self,bounding_box,label_val):
        self.bounding_box = bounding_box
        self.label_val = label_val
    
    @classmethod
    def from_string(cls,str):
        vals = str.split()
        origin = (int(vals[0]),int(vals[1]) )
        width = int(vals[2])
        height = int(vals[3])
        bounding_box = BoundingBox(origin,width,height)
        label_val = vals[4]
        return LabeledExample(bounding_box, label_val)

    def to_string(self):
        return "%d %d %d %d %s"%(
                self.bounding_box.origin[0],
                self.bounding_box.origin[1],
                self.bounding_box.width,
                self.bounding_box.height,
                self.label_val)


def bb_dist(bb, pt):
    (x,y) = bb.origin
    bbpts = [(x,y),(x+bb.width,y),(x,y+bb.height),(x+bb.width,y+bb.height)]
    return min( [(pt[0]-bbpt[0])**2+(pt[1]-bbpt[1])**2 for bbpt in bbpts] )
