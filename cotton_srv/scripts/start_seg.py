#!/usr/bin/env python
import os
import roslib
import rospy
import rospkg
import cv2
import numpy as np
from glob import glob
import argparse
import os
import pickle as pkl
import features_full as ft
import regions as rg
from skimage.measure import regionprops
import time

from get_data import MYDataset
import sys
from std_srvs.srv import SetBool
import threading
import time
from pose_thread import generate_pose
from pose_thread import Concur

#import tensorflow as tf
#from tensorflow.python.keras.backend import set_session
#from tensorflow.python.keras.models import load_model

REGION_AREA = 400
REGION_IGN = 100
INFER_IGN = 100
RAND_SIZE = 30
RATIO = 0.7
model = 'model.p'
data = 'Dataset/tmp'


def feature_and_infer(model, image, regions):
    p = time.time()
    rs = regionprops(regions + 1)
    f, _ = ft.get_features_labels(image, None, train=False, reshape=False)
    results = np.zeros(len(rs), dtype=np.uint8)
    e = enumerate(rs)
    next(e)
    for i, r in e:
        if r.area < INFER_IGN:
            continue
        coords = r.coords
        pop = len(coords)
        n = RAND_SIZE
        if pop > n:
            choices = np.random.choice(pop, size=n, replace=False)
            choices = coords[choices]
            pop = n
        else:
            choices = coords
        # print(f.shape)
        # print(choices.shape)
        X = f[tuple(choices.T)]
        # print(X.shape)
        y = model.predict(X)
        t = np.count_nonzero(y)
        if t / pop > RATIO:
            results[i] = 255
        else:
            results[i] = 0
    p2 = time.time()
    print(p2 - p)
    mask = results[regions]
    return mask

def segment(image, model, raw=False):
    if raw:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)
    p = time.time()
    regions = rg.to_regions(image, REGION_AREA, REGION_IGN)
    p2 = time.time()
    print(p2-p)
    mask = feature_and_infer(model, image, regions)
    return mask

def segment_main(image, raw=False):
    model = pkl.load(open("model.p", "rb"))
    return segment(image, model, raw=False)

# https://stackoverflow.com/questions/50450654/filling-in-circles-in-opencv
def fill(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(mask, contours, -1, 255, thickness=-1)

def get_regions(mask):
    n, labels = cv2.connectedComponents(mask)
    regions = regionprops(labels)
    return [r.coords for r in regions]


class CottonEstimation:
    def __init__(self):
        r = rospkg.RosPack()
        self.pack_path = r.get_path('cotton_srv')
        # load params
        if rospy.has_param('~data'):
            self.img_path = rospy.get_param('~data', "Dataset/tmp")
        else:
            self.img_path = data
        if rospy.has_param('~model'):
            self.model = rospy.get_param('~model', 'model.p')
        else:
            self.model = model
        self.model = os.path.join(r.get_path('cotton_srv'), 'models') + '/' + self.model
        print('model: ', self.model)
        
        ## for ros srv
        #tf_config = tf.ConfigProto()
        #self.sess = tf.Session(config=tf_config)
        #self.graph = tf.get_default_graph()
        #set_session(self.sess)
        
        # DIR to save results
        self.save_dir = os.path.join(r.get_path('cotton_srv'), 'output')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.output = self.save_dir + '/' + 'out.jpg'
        
        # load model
        self.model = pkl.load(open(self.model, "rb"))
        
        # camera parameters
        if rospy.has_param('~intrinsics_fx') and rospy.has_param('~intrinsics_fy') and rospy.has_param(
                '~intrinsics_x0') and rospy.has_param('~intrinsics_y0'):
            self.intrinsics_fx = rospy.get_param('~intrinsics_fx')
            self.intrinsics_fy = rospy.get_param('~intrinsics_fy')
            self.intrinsics_x0 = rospy.get_param('~intrinsics_x0')
            self.intrinsics_y0 = rospy.get_param('~intrinsics_y0')
        else:
            rospy.logerr("Parameters of intrinsics not set.")
        self.intrinsics = np.array(
            [[self.intrinsics_fx, 0.0, self.intrinsics_x0], [0.0, self.intrinsics_fy, self.intrinsics_y0],
             [0., 0., 1.]])
            
        if rospy.has_param('~camera_optical_frame'):
            self.camera_optical_frame = rospy.get_param('~camera_optical_frame', "head_camera_rgb_optical_frame")
        else:
            rospy.logerr("Parameters \'camera_optical_frame\' not set.")

        self.concur = Concur()
        self.first_call = True
        s = rospy.Service('estimate_pose_cotton', SetBool, self.call_srv)

        rospy.loginfo("Estimation initialized.")

    def start_estimation(self):
        n = 1
        while n > 0:
            n = n - 1
            data_path = self.pack_path + '/' + self.img_path
            if not os.path.exists(data_path):
                os.makedirs(data_path)
            dataset_my = MYDataset(data_path)
            
            dep_name = "_depth.png"
            rgb_name = "_color.png"
            rgb_path = os.path.join(data_path, rgb_name)
            
            
            load_data_res = dataset_my.load_data()
            print("Finished load_data")
            if load_data_res:
                image = cv2.imread(rgb_path)
            else:
                rospy.logerr("Load data Failed. Check path of the input images.")
                return False
            print("read ...", rgb_path, flush= True)
            # image = cv2.imread("/home/houjw/cotton/cotton_ws/src/cotton_srv/Dataset/tmp/_color.png")
            
            mask = segment(image, self.model, raw=True)
            #with self.graph.as_default():
                #set_session(self.sess)
                ## star seg
                #mask = segment(image, self.model, raw=True)
            
            cv2.imwrite(self.output, mask)

            ##### some program to get RT of a cotton ###
            RT = np.zeros(( 4, 4))
            tmp_RT = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,3],[0,0,0,1]])
            ##############################
            best_RT = tmp_RT
            self.concur.set_T(best_RT, self.camera_optical_frame)
            if self.first_call:
                self.concur.start()
                self.first_call = False
            else:
                self.concur.resume()



            dataset_my.detect_finished()
        return True
        

    def call_srv(self, req):
        if req.data is False:
            self.concur.pause()
            return [True, "Finished 6D pose estimation service"]
        elif req.data is True:
            res = self.start_estimation()
            return [res, "Started 6D pose estimation service"]

if __name__ == '__main__':
    rospy.init_node('estimate_pose_cotton_server')
    est = CottonEstimation()
    rospy.spin()
