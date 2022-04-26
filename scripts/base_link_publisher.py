#!/usr/bin/env python

from curses.ascii import islower
from operator import is_
import queue
import sys
import math
import json

from matplotlib.transforms import Transform

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from localisation.msg import Landmark, LandmarkArray
from geometry_msgs.msg import Vector3

import unicodedata

import tf2_ros 
from tf.transformations import *
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Pose, PoseStamped, PoseWithCovariance, Transform
from std_msgs.msg import Bool


def updated_tf_callback(msg):
    transformation = TransformStamped()
    # Make tf odom -> base_footprint
    transformation.header = msg.header
    transformation.header.frame_id = odom_tf_name
    transformation.child_frame_id = base_footprint_tf_name

    transformation.transform.translation.x = msg.pose.position.x
    transformation.transform.translation.y = msg.pose.position.y
    transformation.transform.translation.z = 0.

    quaternion = (
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w)
    
    
    euler = euler_from_quaternion(quaternion)

    quaternion = quaternion_from_euler(0., 0., euler[2])
    #type(pose) = geometry_msgs.msg.Pose
    transformation.transform.rotation.x = quaternion[0]
    transformation.transform.rotation.y = quaternion[1]
    transformation.transform.rotation.z = quaternion[2]
    transformation.transform.rotation.w = quaternion[3]

    #transformation.transform.rotation = quaternion_from_euler(0., 0., euler[2])

    trans_broadcaster.sendTransform(transformation)

    # Make tf base_footprint -> base_stabilized
    transformation.header.frame_id = base_footprint_tf_name
    transformation.child_frame_id = base_stabilized_tf_name
    transformation.transform.translation.x = 0.
    transformation.transform.translation.y = 0.
    transformation.transform.translation.z = msg.pose.position.z

    quaternion = quaternion_from_euler(0., 0., 0.)

    transformation.transform.rotation.x = quaternion[0]
    transformation.transform.rotation.y = quaternion[1]
    transformation.transform.rotation.z = quaternion[2]
    transformation.transform.rotation.w = quaternion[3]

    trans_broadcaster.sendTransform(transformation)

    # Make tf base_stabilized -> base_link
    transformation.header.frame_id = base_stabilized_tf_name
    transformation.child_frame_id = base_link_tf_name
    quaternion = quaternion_from_euler(euler[0], euler[1], 0.)
    
    transformation.transform.translation.z = 0

    transformation.transform.rotation.x = quaternion[0]
    transformation.transform.rotation.y = quaternion[1]
    transformation.transform.rotation.z = quaternion[2]
    transformation.transform.rotation.w = quaternion[3]

    trans_broadcaster.sendTransform(transformation)


if __name__ == "__main__":
    odom_tf_name = 'cf1/odom'
    base_footprint_tf_name = 'cf1/base_footprint'
    base_stabilized_tf_name = 'cf1/base_stabilized'
    base_link_tf_name = 'cf1/base_link'
    rospy.init_node('base_link_publisher')
    rospy.logwarn('Initialising base_link_publisher')

    odom_updater = rospy.Subscriber(rospy.get_param('odom_baselink_topic'), PoseStamped, callback=updated_tf_callback)
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    trans_broadcaster  = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        rospy.spin()

        
