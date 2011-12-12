#!/usr/bin/python
import roslib
roslib.load_manifest("hsm_collection")
import cv2
from hsm_collection.bounding_box import BoundingBox,LabeledExample,bb_dist
import os.path
import numpy as np
import random
import IPython

TOP_MARGIN = 60
MARGIN = 60 #Margin in pixels
BPM = 4
SPACE_BETWEEN_MEASURES = 60

NOTE_TYPES  = (WHOLE,   HALF,   QUARTER) = range(3)
BEATS       = (4,       2,      1)
ACCIDENTALS = (NONE,    SHARP,  FLAT) = range(3)
PITCHES     = ('E','F','G','a','b','c','d','e')

#Load the images we care about
TEMPLATES_FOLDER = '../templates'
OUT_FOLDER = '../data/generated'


def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate sheet music')
    parser.add_argument(    '-c','--measure_columns',   dest='measure_cols', type=int,   
                            default=1,
                            help='measure columns' )
    parser.add_argument(    '-r','--measure_rows',   dest='measure_rows', type=int,   
                            default=1,
                            help='measure rows' )
    #parser.add_argument(    '-m','--measure_height',   dest='measure_height', type=int,   
    #                        default=100,
    #                        help='measure height' )
    parser.add_argument(    '-W','--width',  dest='image_width', type=int,
                            default=850,
                            help='width of image.')
    parser.add_argument(    '-H','--image-height',  dest='image_height', type=int,
                            default=1100,
                            help='height of image.')
    parser.add_argument(    '-a','--accidentals',  dest='accidentals',
                            action='store_true', default=False,
                            help='include accidentals (sharp, flat)')
    parser.add_argument(    '-t','--title',  dest='title',
                            action='store_true', default=False,
                            help='include title')
    parser.add_argument(    '-N','--nosave',  dest='nosave',
                            action='store_true', default=False,
                            help='do not save the image')
    parser.add_argument(    '-o','--output_image', dest='output_image',
                            required = True, help='place to save image' )
    return parser.parse_args()

def draw_note(img, note_type, loc, accidental,measure_height):
    """ returns bounding box """
    #For now, just draw a circle
    if note_type == QUARTER:
        templ = cv2.imread('%s/quarter_masked.png'%TEMPLATES_FOLDER,0)
        offset = (20,91)
    elif note_type == HALF:
        templ = cv2.imread('%s/half_masked.png'%TEMPLATES_FOLDER,0)
        offset = (20,91)
    else:
        templ = cv2.imread('%s/whole_masked.png'%TEMPLATES_FOLDER,0)
        offset = (20,98)
    #Rescale, assuming template height is measure height
    scale = measure_height/float(templ.shape[0])
    new_size = (int(templ.shape[1]*scale),int(templ.shape[0]*scale))
    templ_resized = cv2.resize(templ,new_size).astype('uint8')
    off_resized = (offset[0]*scale,offset[1]*scale)
    #Apply the image plus a mask
    tl =     (loc[0]-off_resized[0],
              loc[1]-off_resized[1])
    img[ tl[1]:tl[1]+templ_resized.shape[0],
        tl[0]:tl[0]+templ_resized.shape[1] ] = cv2.min( 
                            img[ tl[1]:tl[1]+templ_resized.shape[0],
                                 tl[0]:tl[0]+templ_resized.shape[1] ],
                            templ_resized )
    bb = BoundingBox(tl,templ_resized.shape[1],templ_resized.shape[0])
    if accidental != NONE:
        if accidental == FLAT:
            atempl = cv2.imread('%s/flat_masked.png'%TEMPLATES_FOLDER,0)
            aoff   = (20,52)
        else:
            atempl = cv2.imread('%s/sharp_masked.png'%TEMPLATES_FOLDER,0)
            aoff   = (20,40)
        atempl_resized = cv2.resize(atempl,(int(scale*atempl.shape[1]),int(scale*atempl.shape[0])) )
        aoff_resized = (aoff[0]*scale,aoff[1]*scale)
        atl =     (loc[0]-off_resized[0] - atempl_resized.shape[1]*0.8,
                   loc[1]-aoff_resized[1])
        img[ atl[1]:atl[1]+atempl_resized.shape[0],
             atl[0]:atl[0]+atempl_resized.shape[1] ] = cv2.min( 
                            img[ atl[1]:atl[1]+atempl_resized.shape[0],
                                 atl[0]:atl[0]+atempl_resized.shape[1] ],
                                 atempl_resized )
        abb = BoundingBox(atl,atempl_resized.shape[1],atempl_resized.shape[0])
    else:
        abb = None
    return bb,abb


def pitch_to_yloc(pitch,measure_height):
    note_spacing = measure_height / 8.
    #Lower notes
    if ord(pitch) in range( ord('A'),ord('G')+1 ):
        return note_spacing*(4 + ord('G')-ord(pitch))
    else:
        return note_spacing*(ord('f')-ord(pitch))

class NoteLabel:
    def __init__(self,type,pitch,accidental,bb,abb):
        self.type = type
        self.pitch = pitch
        self.accidental = accidental
        self.bb = bb
        self.abb = abb


    def to_string(self):
        return "%d %s %d %d %d %d %d"% (
                self.type,self.pitch,self.accidental,self.bb.origin[0],self.bb.origin[1],self.bb.width,self.bb.height)

def main(args):
    #Initialize white image
    im = np.ones((args.image_height,args.image_width),'uint8')*255
    cv2.namedWindow("Image")
    #measure_height = args.measure_height
    measure_height = (args.image_height - TOP_MARGIN)/args.measure_rows - SPACE_BETWEEN_MEASURES
    line_spacing = measure_height / 4.
    note_spacing = measure_height / 8.
    x_range = (MARGIN,args.image_width-MARGIN)
    measure_width = (x_range[1] - x_range[0]) / float(args.measure_cols)
    #measure_width = 200
    print "Measure size: %f x %f"%(measure_width, measure_height)
    beat_width = measure_width / (float(BPM))
    current_height = TOP_MARGIN
    labels = []
    measure_bounds = []
    #Add a title, why not?
    if args.title:
        title_len = random.choice(range(20))+5
        title = ''
        for i in range(title_len):
            title += random.choice([ chr(x) for x in range(ord('A'),ord('z')+1) ])
        cv2.putText(im, title, (MARGIN,0.5*TOP_MARGIN), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,0))
    #Add the filename so we don't forget
    cv2.putText(im, args.output_image, (args.image_width/2,args.image_height-int(0.3*TOP_MARGIN)), cv2.FONT_HERSHEY_SIMPLEX, 0.35,(0,0,0))
    for r in range(args.measure_rows):
        for l in range(5):
            #Draw the staff line
            cv2.line(im,
                    (int(x_range[0]),
                int(current_height+l*line_spacing)),
                (int(x_range[1]),int(current_height+l*line_spacing)),
                (0,0,0))
        #Draw the measure breaks
        y_range = (current_height,current_height+measure_height)
        for m in range(args.measure_cols+1):
            break_x = x_range[0]+m*measure_width
            cv2.line(im,(int(break_x),int(y_range[0])),(int(break_x),int(y_range[1])),
                    (0,0,0) )
            if m < args.measure_cols:
                measure_bounds.append( [(break_x,y_range[0]),(break_x,y_range[1]),
                                       (break_x+measure_width,y_range[0]),(break_x+measure_width,y_range[1])] )
        #Draw notes
        beats_left = BPM*args.measure_cols
        cur_x = x_range[0]+beat_width/2.
        while beats_left > 0:
            while True:
                note_type = random.choice(NOTE_TYPES)
                if BEATS[note_type] <= beats_left:
                    break
            if args.accidentals:
                accidental = random.choice(ACCIDENTALS)
            else:
                accidental = NONE
            pitch = random.choice(PITCHES)
            yloc = pitch_to_yloc(pitch,measure_height)+y_range[0]
            bb,abb = draw_note(im,note_type,(int(cur_x),int(yloc)),accidental,measure_height)
            labels.append(NoteLabel(note_type,pitch,accidental,bb,abb))
            beats_left = beats_left - BEATS[note_type]
            cur_x += beat_width*BEATS[note_type]
        current_height += measure_height + SPACE_BETWEEN_MEASURES
    f = open('./r%dc%d.mb'%(args.measure_rows,args.measure_cols),'w')
    for mb in measure_bounds:
        for pt in mb:
            f.write('%f %f\t'%pt)
        f.write('\n')
    f.close()
    if args.nosave:
        cv2.imshow("Image", im)
        cv2.waitKey(0)
        return
    #Save the image and the label files
    cv2.imwrite(args.output_image, im)
    f = open(args.output_image+'.lab','w')
    for lab in labels:
        f.write("%s/n"%lab.to_string())
    f.close()

if __name__ == '__main__':
    args = parse()
    main(args)

