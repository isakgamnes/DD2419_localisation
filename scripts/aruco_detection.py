#!/usr/bin/env python

import sys
import math
import json
import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped, PoseWithCovariance
from aruco_msgs.msg import Marker, MarkerArray
from crazyflie_driver.msg import Position
import numpy as np

goal_tf = 'map'

def print_marker(marker):
    
    if not tf_buf.can_transform(marker.header.frame_id, goal_tf, marker.header.stamp, rospy.Duration(.1)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % marker.header.frame_id)
        return
    
    m = PoseStamped()
    m.header = marker.header
    id = marker.id
    m.pose = marker.pose.pose
    
    transformed = tf_buf.transform(m, goal_tf)
    pub_aruco_pos.publish(transformed)

    t = TransformStamped()
    t.header.stamp = marker.header.stamp
    t.header.frame_id = transformed.header.frame_id
    t.child_frame_id = 'aruco/detected{}'.format(id)
    t.transform.translation = transformed.pose.position
    t.transform.rotation = transformed.pose.orientation

    trans_broadcaster.sendTransform(t)


def aruco_pos_callback(msg):
    markers = msg.markers
    for marker in markers:
        print_marker(marker)
    
def main():
    rate.sleep()

rospy.init_node('odom_publisher')
sub_aruco_pos = rospy.Subscriber('/aruco/markers', MarkerArray, aruco_pos_callback)
pub_aruco_pos = rospy.Publisher('/aruco/detected_position', PoseStamped)


tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
trans_broadcaster  = tf2_ros.TransformBroadcaster()

if __name__ == "__main__":
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main()
        rate.sleep()