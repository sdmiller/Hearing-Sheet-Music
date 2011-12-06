#!/usr/bin/python
import roslib
roslib.load_manifest("hsm_collection")
import cv2,re,inspect
from hsm_collection.zoom_window import keycommand,ZoomWindow
from hsm_collection.bounding_box import BoundingBox,LabeledExample,bb_dist
import os.path

[CREATE, TRANSLATE, SCALE] = range(3)

class BBLabeler(ZoomWindow):
    def __init__(self, image, label_vals, label_colors, labelpath):
        self.image = image
        self.labelpath = labelpath
        self.labeled_bbs = []
        if os.path.exists(labelpath):
            self.load(labelpath)
        self.label_vals = label_vals
        self.label_colors = label_colors
        self.cur_label_idx = 0
        self.cur_pt = (0,0)
        self.temp_bb = None
        self.select_mode = CREATE
        self.selected_bb = -1
        ZoomWindow.__init__(self,name="Labeler",update_period=100 )
        

    def handleEventsUnzoomed(self, event, x, y, flags, params ):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.select_mode = CREATE
            self.cur_pt = (x,y)
        elif event == cv2.EVENT_LBUTTONUP:
            bb = BoundingBox.from_pts( self.cur_pt,(x,y) )
            self.labeled_bbs.append( LabeledExample( bb, self.get_cur_label() ) )
            self.temp_bb = None
        elif event == cv2.EVENT_MBUTTONDOWN:
            self.selected_bb = self.get_nearest_bb(x,y)
            self.select_mode = TRANSLATE
            self.cur_pt = (x,y)
        elif event == cv2.EVENT_MBUTTONUP or event == cv2.EVENT_RBUTTONUP:
            self.select_mode = CREATE
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.selected_bb = self.get_nearest_bb(x,y)
            self.select_mode = SCALE
            self.cur_pt = (x,y)
        elif event == cv2.EVENT_RBUTTONDBLCLK:
            self.selected_bb = self.get_nearest_bb(x,y)
            del self.labeled_bbs[self.selected_bb]
            self.selected_bb = -1
            self.select_mode = CREATE

        elif flags-32 == cv2.EVENT_FLAG_LBUTTON:
            self.temp_bb = BoundingBox.from_pts( self.cur_pt,(x,y) )
        elif flags-32 == cv2.EVENT_FLAG_MBUTTON and self.select_mode == TRANSLATE:
            dx = x - self.cur_pt[0]
            dy = y - self.cur_pt[1]
            self.labeled_bbs[self.selected_bb].bounding_box.shift(dx,dy)
            self.cur_pt = (x,y)
        elif flags-32 == cv2.EVENT_FLAG_RBUTTON and self.select_mode == SCALE:
            dx = x - self.cur_pt[0]
            dy = y - self.cur_pt[1]
            self.labeled_bbs[self.selected_bb].bounding_box.width += dx
            self.labeled_bbs[self.selected_bb].bounding_box.height += dy
            self.cur_pt = (x,y)

            
    def get_nearest_bb(self,x,y):
        return min(range(len(self.labeled_bbs)),
                key = lambda l: bb_dist(self.labeled_bbs[l].bounding_box,(x,y)) )

    def get_cur_label(self):
        return self.label_vals[self.cur_label_idx]

    @keycommand('=', "Increment label value", exits=False )
    def increment_cur_label(self):
        self.cur_label_idx = (self.cur_label_idx + 1) % len(self.label_vals)
        print "On label: %s"%self.get_cur_label()

    @keycommand('-', "Decrement label value", exits=False )
    def decrement_cur_label(self):
        self.cur_label_idx = (self.cur_label_idx - 1) % len(self.label_vals)
        print "On label: %s"%self.get_cur_label()

    @keycommand('s', "Save Label Values", exits=True )
    def save(self):
        f = open(self.labelpath,'w')
        for l in self.labeled_bbs:
            f.write(l.to_string())
            f.write("\n")
        f.close()

    def load(self,path):
        f = open(path,'r')
        for ln in f.readlines():
            lbb = LabeledExample.from_string(ln)
            self.labeled_bbs.append(lbb)
        f.close()

    def image_to_show(self):
        imcpy = self.image.copy()
        for l in self.labeled_bbs:
            color = self.label_colors[self.label_vals.index(l.label_val)]
            l.bounding_box.draw_to_image(imcpy,color,thickness=1)
        # If there's a temp bounding box, draw it
        if self.temp_bb:
            self.temp_bb.draw_to_image(imcpy,(100,100,100),thickness=1)
        return imcpy

def main(ags):
    image = cv2.imread( args.image )
    labelpath = "%s.lab"%args.image
    label_colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(0,255,255),(255,0,255)]
    bbl = BBLabeler(image, args.label_values,label_colors[:len(args.label_values)], labelpath)


def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Label bounding boxes in an image')
    parser.add_argument(    '-i','--input-image',   dest='image', type=str,   
                            required=True,
                            help='the image to label' )
    parser.add_argument(    '-l','--label-values',  dest='label_values', type=str,
                            required=True, nargs='+' )
    return parser.parse_args()

if __name__ == '__main__':
    args = parse()
    main(args)
