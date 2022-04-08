#!/usr/bin/env python

import queue
import sys
import math
import json
from cv2 import transform
import copy

from torch import inverse
import rospy
import tf2_ros 
from tf.transformations import * #quaternion_from_euler, euler_from_quaternion, compose_matrix, inverse_matrix, decompose_matrix
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3, Pose, PoseStamped, PoseWithCovariance, Transform
from aruco_msgs.msg import Marker, MarkerArray
from crazyflie_driver.msg import Position
import numpy as np
from localisation.msg import Landmark, LandmarkArray
import transformation_functions

goal_tf = 'map'
drone_position = PoseStamped()
init_localisation = True
drone_position_updated = False
landmarks_registrered = False
landmarks = LandmarkArray().landmarks

def data_assoc(transformed, id):
    global landmarks, init_localisation, landmarks_registrered
    if landmarks_registrered:
        # Check wether or not to use the id to associate
        if init_localisation:
            for landmark in landmarks:
                # Check if the id of the stored landmarks correspond to the detected
                if landmark.id == id and landmark.name == 'ArucoMarker':
                    # Tell the system not to use ID in next iteration
                    init_localisation = False
                    return landmark  
        else:
            shortest_dist = float('inf')
            for landmark in landmarks:
                # Check that we are testing for an Aruco Marker
                if landmark.name == 'ArucoMarker':
                    position = [landmark.pose.position.x, landmark.pose.position.y, landmark.pose.position.z]
                    # Calculate the distance to the current Aruco Marker
                    lm_dist_sq =(position[0] - transformed.pose.position.x)**2 + \
                                (position[1] - transformed.pose.position.y)**2 + \
                                (position[2] - transformed.pose.position.z)**2
                    # If the distance is shorter than the shorest distance, store this marker
                    if lm_dist_sq < shortest_dist:
                        shortest_dist = lm_dist_sq
                        assoc_lm = landmark

        return assoc_lm
    else: 
        return None

# Transform drone pos to position in the detected aruco marker
# Compare the difference
# Diff = odom_pos



def estimate_odom_pos(marker):
    global trans_broadcaster
    # Figure out which marker we are looking at
    
    observed_lm = data_assoc(marker.pose, marker.id)

    if observed_lm == None:
        return

    # Create a TransformStamped for the observed lm
    t = TransformStamped()

    # Give it the same header as the observed landmark
    t.header = marker.header

    # Give a suitable name to the detected marker's tf. Using the id of the associated landmark
    t.child_frame_id = 'aruco/detected{}'.format(observed_lm.id)

    # Set the pose and orientation equal to the detected marker pose and orientation
    t.transform.translation = marker.pose.pose.position
    t.transform.rotation = marker.pose.pose.orientation

    # Send the transform
    trans_broadcaster.sendTransform(t)

    # Request the transform from Source: 'map' to Target: 'aruco/markerX'
    mTa = tf_buf.lookup_transform('map', 'aruco/marker{}'.format(observed_lm.id), rospy.Duration(.1))
    # Request the transform from Source: 'aruco/detectedX' to Target: 'map'
    dTm = tf_buf.lookup_transform('map', t.child_frame_id, marker.header.stamp, timeout=rospy.Duration(.2))
    
    # Find the old odom in map
    old_odom = tf_buf.lookup_transform('map', 'cf1/odom', marker.header.stamp, timeout=rospy.Duration(.2))
    
    odom_diff = transformation_functions.transform_diff(dTm.transform, mTa.transform)

    new_x = old_odom.transform.translation.x + odom_diff.translation.x
    new_y = old_odom.transform.translation.y + odom_diff.translation.y

    
    # Create the tranformation from 'map' to 'cf1/odom'
    odom = TransformStamped()
    odom.header.stamp = marker.header.stamp
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'cf1/odom'
    odom.transform.translation.x = new_x
    odom.transform.translation.y = new_y
    odom.transform.translation.z = 0.

    """quaternion = (
    map2odom.rotation.x,
    map2odom.rotation.y,
    map2odom.rotation.z,
    map2odom.rotation.w)"""
    quaternion = (
    0.,0.,0.,1.)

    euler = euler_from_quaternion(quaternion)

    quaternion = quaternion_from_euler(0., 0., euler[2])

    odom.transform.rotation.x = quaternion[0]
    odom.transform.rotation.y = quaternion[1]
    odom.transform.rotation.z = quaternion[2]
    odom.transform.rotation.w = quaternion[3]

    odom_updated.publish(odom)

    #print(odom)



def estimate_drone_position(marker):
    global init_localisation
    if not tf_buf.can_transform(marker.header.frame_id, goal_tf, marker.header.stamp, rospy.Duration(.3)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % marker.header.frame_id)
        estimate_odom_pos(marker)
        return
    
    m = PoseStamped()
    m.header = marker.header
    id = marker.id
    m.pose = marker.pose.pose
    
    # Transform the detected marker to map frame
    transformed = tf_buf.transform(m, goal_tf)

    # data association
    assoc_lm = data_assoc(transformed, id)
    if assoc_lm == None:
        return
    #print(assoc_lm)

    # Create a transformation
    t = TransformStamped()
    t.header.stamp = marker.header.stamp
    t.header.frame_id = transformed.header.frame_id
    t.child_frame_id = 'aruco/detected{}'.format(id)
    t.transform.translation = transformed.pose.position
    t.transform.rotation = transformed.pose.orientation

    trans_broadcaster.sendTransform(t)
    



def store_landmarks(msg):
    global landmarks, landmarks_registrered
    # Store the incoming list of landmarks globally
    landmarks = msg.landmarks
    landmarks_registrered = True

def aruco_pos_callback(msg):
    markers = msg.markers
    # Iterate through all the detected markers in the image
    for marker in markers:
        estimate_odom_pos(marker)

def drone_position_callback(msg):
    global drone_position, drone_position_updated
    # Store the position of the drone defined in the odom frame globally
    drone_position = msg
    drone_position_updated = True


def main():
    rate.sleep()

rospy.init_node('localisation')
sub_aruco_pos = rospy.Subscriber('/aruco/markers', MarkerArray, aruco_pos_callback, queue_size=1)
sub_drone_pos = rospy.Subscriber('/cf1/pose', PoseStamped, drone_position_callback)
marker_array = rospy.Subscriber('landmarks', LandmarkArray, store_landmarks)
odom_updated = rospy.Publisher('/loc/odom_est', TransformStamped, queue_size=10)


tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
trans_broadcaster  = tf2_ros.TransformBroadcaster()


if __name__ == "__main__":
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main()
        rate.sleep()