#!/usr/bin/env python

from curses.ascii import islower
from operator import is_
import queue
import sys
import math
import json

from matplotlib.transforms import Transform
from genpy import Duration

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from localisation.msg import Landmark, LandmarkArray
from geometry_msgs.msg import Vector3

import unicodedata

import tf2_ros 
import tf2_geometry_msgs
from tf.transformations import *
from geometry_msgs.msg import Vector3, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool


def updated_tf_callback(msg):
    if not tf_buf.can_transform(msg.header.frame_id, "map", msg.header.stamp, timeout=rospy.Duration(.2)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % msg.header.frame_id)
        return
    
    pose_in_map = tf_buf.transform(msg, 'map', rospy.rostime.Duration(.1))
    
    measurement = PoseStamped()
    measurement.header = pose_in_map.header
    measurement.pose = pose_in_map.pose
    
    measurement_publisher.publish(measurement)
    

if __name__ == "__main__":
    ekf_pose_topic = '/ekf/cf1_pose'
    rospy.init_node('ekf_pose_control')
    rospy.logwarn('Initialising {}'.format(rospy.get_name()))

    pose_updater = rospy.Subscriber(rospy.get_param('odom_baselink_topic'), PoseStamped, callback=updated_tf_callback)
    measurement_publisher = rospy.Publisher(ekf_pose_topic, PoseStamped)
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    trans_broadcaster  = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        rospy.spin()
