#!/usr/bin/env python

import queue
import sys
import math
import json
from cv2 import Mahalanobis, transform
import copy

from torch import inverse
from genpy import Duration
import rospy
import tf2_ros 
from tf.transformations import * #quaternion_from_euler, euler_from_quaternion, compose_matrix, inverse_matrix, decompose_matrix
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3, Pose, PoseStamped, PoseWithCovarianceStamped, Transform
from aruco_msgs.msg import Marker, MarkerArray
from crazyflie_driver.msg import Position
import numpy as np
from localisation.msg import Landmark, LandmarkArray
import transformation_functions
from scipy.spatial.distance import mahalanobis

goal_tf = 'map'
drone_position = PoseStamped()
init_localisation = True
drone_position_updated = False
landmarks_registrered = False
landmarks = LandmarkArray().landmarks

def data_assoc(marker):
    global landmarks, init_localisation, landmarks_registrered
    assoc_lm = None
    if landmarks_registrered:
        # Check wether or not to use the id to associate
        if init_localisation:
            for landmark in landmarks:
                if landmark.name == 'ArucoMarker':
                    # Check if the id of the stored landmarks correspond to the detected
                    if landmark.id == marker.id:
                        # Tell the system not to use ID in next iteration
                        return landmark  
        else:
            if not tf_buf.can_transform('map', marker.header.frame_id, marker.header.stamp, timeout=rospy.Duration(.1)):
                rospy.logwarn_throttle(1, 'Cannot transform from {} to map frame..'.format(marker.header.frame_id))
                return
        
            observed_marker_xyz = np.array([marker.pose.position.x, marker.pose.position.y,\
                                    marker.pose.position.z])
            # Normalize the xyz coordinates
            observed_marker_xyz = observed_marker_xyz/np.linalg.norm(observed_marker_xyz)

            observed_marker_rpy =  np.array([marker.pose.orientation.x, marker.pose.orientation.y,\
                                    marker.pose.orientation.z, marker.pose.orientation.w])
            
            observed_marker = np.concatenate((observed_marker_xyz, observed_marker_rpy))

            shortest_dist = float('inf')
            for landmark in landmarks:
                # Check that we are testing for an Aruco Marker
                if landmark.name == 'ArucoMarker':
                    marker_xyz = np.array([landmark.pose.position.x, landmark.pose.position.y, \
                                           landmark.pose.position.z])
                    marker_xyz = marker_xyz/np.linalg.norm(marker_xyz)

                    marker_rpy = np.array([landmark.pose.orientation.x, landmark.pose.orientation.y,\
                                           landmark.pose.orientation.z, landmark.pose.orientation.w])
                    
                    marker_pose = np.concatenate((marker_xyz, marker_rpy))
                    
                    lm_dist = np.linalg.norm(observed_marker - marker_pose)
                    # Calculate the distance to the current Aruco Marker
                    #rospy.logwarn_throttle(-1, 'Marker with id: {}, Normalized distance: {}'.format(landmark.id, lm_dist))
                    #rospy.logwarn_throttle(-1, 'ID: {}, position: {}'.format(landmark.id,position))
                    # If the distance is shorter than the shorest distance, store this marker
                    if lm_dist < shortest_dist:
                        shortest_dist = lm_dist
                        assoc_lm = landmark
        return assoc_lm
    else: 
        return None

# Transform drone pos to position in the detected aruco marker
# Compare the difference
# Diff = odom_pos



def estimate_odom_pos(marker):
    global trans_broadcaster, drone_position, init_localisation

    # Create Pose object from the detected marker
    marker_pose = PoseStamped()
    marker_pose.header = marker.header
    marker_pose.pose = marker.pose.pose

    # Get the transformation from map to the camera frame
    map_camera_transformation = tf_buf.lookup_transform('map', marker.header.frame_id, marker.header.stamp, rospy.Duration(.1))
    # Get the transformation from odom to the camera frame
    odom_camera_transformation = tf_buf.lookup_transform('cf1/odom', marker.header.frame_id, marker.header.stamp, rospy.Duration(.1))
    
    # Convert the detected marker into the map frame
    marker_in_map = tf2_geometry_msgs.do_transform_pose(marker_pose, map_camera_transformation)
    # Convert the detected marker into the odom frame
    marker_in_odom = tf2_geometry_msgs.do_transform_pose(marker_pose, odom_camera_transformation)

    if init_localisation:
        # Figure out which marker we are looking at
        observed_lm = data_assoc(marker)
    else:
        observed_lm = data_assoc(marker_in_map)
    # Check if it managed to associate
    if observed_lm == None:
        return
    
    # DEBUG: print wether or not it was the correct marker
    if observed_lm.id == marker.id:
        rospy.logwarn('Associated correct marker')
    else:
        rospy.logwarn('Associated wrong marker')

    # Get the marker position and orientation in the odom frame
    marker_position = marker_in_odom.pose.position
    marker_orientation = marker_in_odom.pose.orientation

    # Create a quaternion matrix from the detected marker and invert it
    marker_odom_transformation = inverse_matrix(quaternion_matrix([marker_orientation.x, marker_orientation.y, \
                                                                      marker_orientation.z, marker_orientation.w]))
    
    # Define the translation in the transformation matrix
    marker_odom_transformation[0:4, 3] = np.matmul(marker_odom_transformation, np.array([-marker_position.x,\
                                                                                         -marker_position.y,\
                                                                                         -marker_position.z,\
                                                                                             1]))

    # Create matrix from the associated marker
    detected_marker_position = np.array([observed_lm.pose.position.x, observed_lm.pose.position.y, observed_lm.pose.position.z])
    
    observed_lm_in_map = quaternion_matrix([observed_lm.pose.orientation.x, observed_lm.pose.orientation.y,\
                                            observed_lm.pose.orientation.z, observed_lm.pose.orientation.w])
    observed_lm_in_map[0:3, 3] = detected_marker_position

    # Calculate the odom position based on the observed landmark
    odom_in_map = np.matmul(observed_lm_in_map, marker_odom_transformation)
    odom_orientation = euler_from_matrix(odom_in_map)
    
    # Create the tranformation from 'map' to 'cf1/odom'
    odom = PoseStamped()
    odom.header.stamp = marker.header.stamp
    odom.header.frame_id = 'map'
    odom.pose.position.x = odom_in_map[0,3]
    odom.pose.position.y = odom_in_map[1,3]
    odom.pose.position.z = 0.

    (odom.pose.orientation.x,
    odom.pose.orientation.y,
    odom.pose.orientation.z,
    odom.pose.orientation.w) = quaternion_from_euler(0.,0.,odom_orientation[2])

    if not init_localisation:
        odom_updated.publish(odom)

    else:
        odom_publisher.publish(odom)
        #trans_broadcaster.sendTransform(odom)
        init_localisation = False


def sixD_pose_callback(landmark):
    rospy.logwarn_throttle(5, 'Got traffic sign lm')
    transform = TransformStamped()
    transform.transform.translation = landmark.pose.position
    transform.transform.rotation = landmark.pose.orientation
    transform.header = landmark.header

    transform.child_frame_id = landmark.name

    trans_broadcaster.sendTransform(transform)

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
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
trans_broadcaster  = tf2_ros.TransformBroadcaster()
rospy.sleep(rospy.Duration(.1))
sub_aruco_pos = rospy.Subscriber('/aruco/markers', MarkerArray, aruco_pos_callback, queue_size=1)
sub_traffic_pos = rospy.Subscriber('/6D_sign', Landmark, sixD_pose_callback, queue_size=1)
sub_drone_pos = rospy.Subscriber('/cf1/pose', PoseStamped, drone_position_callback)
marker_array = rospy.Subscriber('landmarks', LandmarkArray, store_landmarks)
odom_updated = rospy.Publisher('/ekf/cf1_measurement', PoseStamped, queue_size=10)
odom_publisher = rospy.Publisher('/loc/odom_est', PoseStamped, queue_size=10)

measurement_covariance = [  0.1, 0,    0,    0,    0,    0,    
                                0,    0.1, 0,    0,    0,    0,    
                                0,    0,    0,    0,    0,    0,    
                                0,    0,    0,    0,    0,    0,    
                                0,    0,    0,    0,    0,    0,    
                                0,    0,    0,    0,    0,    0.1]

if __name__ == "__main__":
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main()
        rate.sleep()