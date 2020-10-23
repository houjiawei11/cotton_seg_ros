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
import time
import inference_seg as infseg

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
# model = 'model.p'
data = 'Dataset/tmp'

# https://github.com/hughw19/NOCS_CVPR2019/blob/utils.py#L3017
def get_centroid(depth, intrinsics, instance_mask):
    intrinsics_inv = np.linalg.inv(intrinsics)
    image_shape = depth.shape
    width = image_shape[1]
    height = image_shape[0]

    x = np.arange(width)
    y = np.arange(height)

    #non_zero_mask = np.logical_and(depth > 0, depth < 5000)
    non_zero_mask = (depth > 0)
    final_instance_mask = np.logical_and(instance_mask, non_zero_mask)

    idxs = np.where(final_instance_mask)
    grid = np.array([idxs[1], idxs[0]])

    # shape: height * width
    # mesh_grid = np.meshgrid(x, y) #[height, width, 2]
    # mesh_grid = np.reshape(mesh_grid, [2, -1])
    length = grid.shape[1]
    ones = np.ones([1, length])
    uv_grid = np.concatenate((grid, ones), axis=0) # [3, num_pixel]

    xyz = intrinsics_inv @ uv_grid # [3, num_pixel]
    xyz = np.transpose(xyz) #[num_pixel, 3]

    z = depth[idxs[0], idxs[1]]

    # print(np.amax(z), np.amin(z))
    pts = xyz * z[:, np.newaxis]/xyz[:, -1:]
    pts[:, 0] = -pts[:, 0]
    pts[:, 1] = -pts[:, 1]

    return pts.mean(axis=0)


class CottonEstimation:
    def __init__(self):
        r = rospkg.RosPack()
        self.pack_path = r.get_path('cotton_srv')
        # load params
        if rospy.has_param('~data'):
            self.img_path = rospy.get_param('~data', "Dataset/tmp")
        else:
            self.img_path = data
        # if rospy.has_param('~model'):
        #     self.model = rospy.get_param('~model', 'model.p')
        # else:
        #     self.model = model
        # self.model = os.path.join(r.get_path('cotton_srv'), 'models') + '/' + self.model
        # print('model: ', self.model)

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
        # self.model = pkl.load(open(self.model, "rb"))

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
            dep_path = os.path.join(data_path, dep_name)


            load_data_res = dataset_my.load_data()
            print("Finished load_data")
            if load_data_res:
                image = cv2.imread(rgb_path)
                depth = cv2.imread(dep_path, -1)
            else:
                rospy.logerr("Load data Failed. Check path of the input images.")
                return False
            print("read ...", rgb_path, flush=True)
            print("read ...", dep_path, flush=True)

            # image = cv2.imread("/home/houjw/cotton/cotton_ws/src/cotton_srv/Dataset/tmp/_color.png")

            mask = infseg.segment_one(image, depth, raw=True)
            #with self.graph.as_default():
                #set_session(self.sess)
                ## star seg
                #mask = segment(image, self.model, raw=True)

            cv2.imwrite(self.output, mask)

            ##### some program to get RT of a cotton ###
            c = get_centroid(depth, self.intrinsics, mask)

            # RT = np.zeros(( 4, 4))
            # tmp_RT = np.array([ [1,0,0,0], \
            #                   [0,1,0,-0.2], \
            #                   [0,0,1,0.5], \
            #                   [0,0,0,1]])
            # best_RT = tmp_RT
            # ##############################
            best_RT = np.identity(4)
            best_RT[0:3, 3] = c

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
