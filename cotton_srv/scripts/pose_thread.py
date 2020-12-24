#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf
import tf.transformations as tft
import threading
import time
import numpy

def generate_pose(T, tf_frame):
    print('publish TF for \n', T)
    rate = rospy.Rate(30)  # Hz

    p = Pose()
    p.position.x = T[0, 3]
    p.position.y = T[1, 3]
    p.position.z = T[2, 3]
    R = T[:3, :3]
    roll, pitch, yaw = tft.euler_from_matrix(T)
    q = tft.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    #tf_brocast(p, camera_optical_frame)
    tf_brocast(p, tf_frame)


def tf_brocast(p, frame_id):
    br = tf.TransformBroadcaster()
    br.sendTransform((p.position.x, p.position.y, p.position.z), \
                     (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), \
                     rospy.Time.now(), "object_predicted", frame_id)


class Concur(threading.Thread):
    def __init__(self):
        super(Concur, self).__init__()
        self.iterations = 0
        self.daemon = True  # Allow main to exit even if still running.
        self.paused = True  # Start out paused.
        self.state = threading.Condition()

        self.listener = tf.TransformListener()

    def set_T(self, T, camera_optical_frame):

        listener = self.listener

        listener.waitForTransform('/base_link', camera_optical_frame, rospy.Time(), rospy.Duration(4.0))
        tf_flag = False
        while not tf_flag:
            try:
                now = rospy.Time.now()
                listener.waitForTransform('/base_link', camera_optical_frame, now, rospy.Duration(4.0))
                (t,r) = listener.lookupTransform('/base_link', camera_optical_frame, now)
                tf_flag = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("failed to listen transform from '/base_link' to '{}'".format(camera_optical_frame))
        cam2base = numpy.matrix(tft.quaternion_matrix(r))
        cam2base[0,3] = t[0]
        cam2base[1,3] = t[1]
        cam2base[2,3] = t[2]
        
        T = numpy.array(numpy.dot(cam2base, T))
        self.T = T
        self.tf_frame = "base_link"

    def run(self):
        self.resume()
        while True:
            with self.state:
                if self.paused:
                    self.state.wait()  # Block execution until notified.
            generate_pose(self.T, self.tf_frame)
            # time.sleep(2.5)

    def resume(self):
        with self.state:
            self.paused = False
            self.state.notify()  # Unblock self if waiting.

    def pause(self):
        with self.state:
            self.paused = True  # Block self.
            print("\n Pause sending Transform.")
