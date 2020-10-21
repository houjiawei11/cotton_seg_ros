 #!/usr/bin/env python  
import os
import sys
import datetime
import time
import roslib
import cv2
import rospy
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from distutils.version import LooseVersion
import pandas as pd
import numpy as np


input_flag = False
#width = 640
#height = 480
#dsize = (width, height)

class MYDataset:
    """Generates the NOCS dataset.
    """
    
    def __init__(self, image_dir, dep_name = "_depth.png",  rgb_name = "_color.png"):
        # dir to save real-time RGBD images
        self.image_dir = image_dir
        self.dep_name = dep_name
        self.rgb_name = rgb_name

        if rospy.has_param('~rgb_topic') and rospy.has_param('~dep_topic'):
            self.dep_topic = rospy.get_param('~dep_topic')
            self.rgb_topic = rospy.get_param('~rgb_topic')
        else:
            rospy.logerr("RGBD camera topics not set.")
        
    
    def callback(self, imaged, imagec):
        global input_flag
        if input_flag:
            return
        try:
            NewImg = None
            if imaged.encoding == "32FC1":
                cv_image = self.CvImg.imgmsg_to_cv2(imaged, "32FC1")
                print("Depth image encoding: 32FC1")
                image = np.array(cv_image, dtype=np.float)
                image = image * 1000  # unit: m to mm
                NewImg = np.round(image).astype(np.uint16)
            elif imaged.encoding == "16UC1":
                NewImg = self.CvImg.imgmsg_to_cv2(imaged, "16UC1")
                print("Depth image encoding: 16UC1")
            else:
                rospy.logerr("Depth image encoding is neither 32FC1 nor 16UC1")
                return
            img_path = os.path.join(self.image_dir, self.dep_name)
            cv2.imwrite(img_path, NewImg)
            print("saved ", self.dep_name)
        except CvBridgeError as e:
            print(e)
        try:
            NewImgd = self.CvImg.imgmsg_to_cv2(imagec, "bgr8")
            img_path = os.path.join(self.image_dir, self.rgb_name)
            cv2.imwrite(img_path, NewImgd)
            print("saved ", self.rgb_name)
        except CvBridgeError as e1:
            print(e1)
        input_flag = True
        
    def load_data(self):
        self.CvImg = CvBridge()
        skDep = message_filters.Subscriber(self.dep_topic, Image)
        skImg = message_filters.Subscriber(self.rgb_topic, Image)
        # ats = message_filters.ApproximateTimeSynchronizer(list([skDep, skImg]), queue_size=5, slop=0.1)
        # ats.registerCallback(self.callback)
        ts = message_filters.TimeSynchronizer([skDep, skImg], 10)
        ts.registerCallback(self.callback)
        # skImg = rospy.Subscriber(self.rgb_topic, Image, self.callbackImg)
        # rospy.spin()

        
        i = 0
        while not input_flag:
            time.sleep(2.5)
            i = i + 1
            print("Waiting rgbd topics ... ", i)
            if i > 9:
                print("no dep/rgb image subcribed for 25 seconds: exit the detection")
                return False
        return True
    
    def detect_finished(self):
        global input_flag
        input_flag = False
