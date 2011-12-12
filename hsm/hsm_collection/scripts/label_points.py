#!/usr/bin/python
import roslib
roslib.load_manifest("hsm_collection")
import cv2,re,inspect
from hsm_collection.zoom_window import keycommand,ZoomWindow
from hsm_collection.bounding_box import BoundingBox,LabeledExample,bb_dist
import os.path


class PointLabeler(ZoomWindow):
    def __init__(self, image, labelpath):
        self.image = image
        self.labelpath = labelpath
        self.labeled_pts = []
        if os.path.exists(labelpath):
            self.load(labelpath)
        self.rad = 2
        ZoomWindow.__init__(self,name="Labeler",update_period=100, zoom_out=2 )
        

    def handleEventsUnzoomed(self, event, x, y, flags, params ):
        if event == cv2.EVENT_LBUTTONUP:
            self.labeled_pts.append((int(x),int(y)))
        elif event == cv2.EVENT_RBUTTONDBLCLK:
            selected = self.get_nearest_pt(x,y)
            del self.labeled_pts[selected]

    def get_nearest_pt(self,x,y):
        return min(range(len(self.labeled_pts)),
                key = lambda l: (self.labeled_pts[l][0]-x)**2+(self.labeled_pts[l][1]-y)**2 )
    @keycommand('=', "Increment Radius")
    def incr_rad(self):
        self.rad += 1

    @keycommand('-', "decrement Radius")
    def decr_rad(self):
        self.rad -= 1

    @keycommand('s', "Save Label Values", exits=True )
    def save(self):
        f = open(self.labelpath,'w')
        for pt in self.labeled_pts:
            f.write("%d %d\n"%pt)
        f.close()

    def load(self,path):
        f = open(path,'r')
        for ln in f.readlines():
            v = ln.split()
            pt = (int(v[0]),int(v[1]))
            self.labeled_pts.append(pt)
        f.close()

    def image_to_show(self):
        imcpy = self.image.copy()
        for pt in self.labeled_pts:
            cv2.circle(imcpy,pt,self.rad,(255,0,0),-1)
        return imcpy

def main(ags):
    image = cv2.imread( args.image )
    labelpath = "%s.pts"%args.image
    ptl = PointLabeler(image, labelpath)


def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Label bounding boxes in an image')
    parser.add_argument(    '-i','--input-image',   dest='image', type=str,   
                            required=True,
                            help='the image to label' )
    return parser.parse_args()

if __name__ == '__main__':
    args = parse()
    main(args)
