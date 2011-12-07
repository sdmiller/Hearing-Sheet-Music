#!/usr/bin/python
import roslib
roslib.load_manifest("hsm_collection")
import cv2,re,inspect
from hsm_collection.bounding_box import BoundingBox,LabeledExample,bb_dist
import os.path
import PyML
import PyML.classifiers.multi
import numpy as np
import IPython

def parse():
    import argparse
    
    parser = argparse.ArgumentParser(description='Label bounding boxes in an image')
    parser.add_argument(    '-i','--images',   dest='images', type=str,   
                            required=True, nargs='+',
                            help='the image to label' )
    parser.add_argument(    '-c','--classes',  dest='classes', type=str,
                            required=True, nargs='+',
                            help='classes you wish to consider.')
    parser.add_argument(    '-n','--negative-ratio',  dest='negative_ratio', type=float,
                            default=0,
                            help='amount of random negatives to consider, as a percentage of total positive examples')
    parser.add_argument(    '-f','--feature',  dest='feature', type=str,
                            default='bw', choices = ['bw','color'],
                            help='feature to use')
    parser.add_argument(    '-m','--model_file',  dest='model_file', type=str,
                            required=True,
                            help='feature to use')
    return parser.parse_args()

def compute_feature( patch, feature_type ):
    if feature_type == 'bw':
        return patch.flatten().astype(float)
    else:
        assert False


def main(args):

    X = np.array([])
    patches = []
    neg_patches = []
    l = []
    #First pass through the images and collect the examples
    for image_name in args.images:
        image = cv2.imread( image_name, 0 )
        #Open the label file
        f = open(image_name+'.lab')
        for ln in f.readlines():
            lbb = LabeledExample.from_string(ln)
            if lbb.label_val in args.classes:
                patch = lbb.bounding_box.extract(image)
                patches.append(patch)
                l.append(lbb.label_val)
            elif len(neg_patches) < len(patches)*args.negative_ratio:
                patch = lbb.bounding_box.extract(image)
                neg_patches.append(patch)
        f.close()
    
    if os.path.exists(args.model_file):
        #If the model already exists, we use the learned filter size
        f = open(args.model_file+'.winsize')
        ln = f.readline()
        mean_width = int(ln[0])
        mean_height = int(ln[1])
        f.close()
    else:
        #Otherwise we compute the mean ourselves
        mean_width = int(sum([p.shape[0] for p in patches])/float(len(patches)))
        mean_height = int(sum([p.shape[1] for p in patches])/float(len(patches)))
    l = l + ['NEGATIVE' for n in neg_patches]
    for i,patch in enumerate(patches + neg_patches):
        patch_scaled = cv2.resize( patch, (mean_width, mean_height) )
        feature = compute_feature( patch_scaled, args.feature )
        if i == 0:
            X = feature
        else:
            X = np.vstack( (X, feature))
    L = PyML.Labels(l)
    print L
    data = PyML.VectorDataSet(X,L=L)
    #Train if the model doesn't exist
    if not os.path.exists(args.model_file):
        mc = PyML.classifiers.multi.OneAgainstRest(PyML.svm.SVM())
        param = PyML.modelSelection.Param(mc, 'C', [0.1, 1, 100, 1000])
        m = PyML.modelSelection.ModelSelector(param)
        m.train(data)
        cl = m.classifier
        cl.loo(data)
        #Saving data since for some absurd reason we need it
        data.save(args.model_file+'.data')
        cl.save(args.model_file)
        #Saving the window size...I should really wrap all of this
        f = open(args.model_file+'.winsize','w')
        f.write('%d\t%d'%(mean_width,mean_height))
        f.close()
    else:
        #Otherwise test
        mc = PyML.classifiers.multi.OneAgainstRest(PyML.svm.SVM())
        train_data = PyML.VectorDataSet(args.model_file+'.data',labelsColumn=1)
        mc.load(args.model_file,train_data)
        print mc.test(data)
if __name__ == '__main__':
    args = parse()
    main(args)
