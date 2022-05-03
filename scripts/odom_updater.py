#!/usr/bin/env python

import queue
import sys
import math
import json
from cv2 import transform
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
            
            marker_stamped = PoseStamped()

            marker_stamped.header = marker.header
            marker_stamped.pose.position = marker.pose.pose.position
            marker_stamped.pose.orientation = marker.pose.pose.orientation

            transformed = tf_buf.transform(marker_stamped, 'map')
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
    global trans_broadcaster, drone_position, init_localisation

    #rospy.logwarn_throttle(1, marker)

    # Create a TransformStamped for the observed lm
    detected_from_drone = TransformStamped()

    # Give it the same header as the observed landmark
    detected_from_drone.header = marker.header

    # Give a suitable name to the detected marker's tf. Using the id of the associated landmark
    detected_from_drone.child_frame_id = 'aruco/detected{}'.format(marker.id)

    # Set the pose and orientation equal to the detected marker pose and orientation
    detected_from_drone.transform.translation = marker.pose.pose.position
    detected_from_drone.transform.rotation = marker.pose.pose.orientation

    # Send the transform
    trans_broadcaster.sendTransform(detected_from_drone)

    # Figure out which marker we are looking at
    observed_lm = data_assoc(marker)
    
    if observed_lm == None:
        return

    # rospy.logwarn_throttle(-1, marker)

    """if observed_lm.id == marker.id:
        rospy.logwarn_throttle(-1, 'Associated the correct marker!')
    else:
        rospy.logwarn_throttle(-1, 'Associated wrong marker..')
    rospy.logwarn_throttle(-1, 'Associated marker id: {}'.format(observed_lm.id))"""

    # Request the transform from Source: 'aruco/markerX' to Target: 'map'
    mTa = tf_buf.lookup_transform('map', 'aruco/marker{}'.format(observed_lm.id), time=rospy.Time(0), timeout=rospy.Duration(.1))

    # mTa_homogeneous = transformation_functions.transform2homogeneousM(mTa.transform)
    # detected_from_drone_homogeneous = transformation_functions.transform2homogeneousM(detected_from_drone.transform)

    # drone_from_detected_homogeneous = inverse_matrix(detected_from_drone_homogeneous)
    # drone_in_map_homogeneous = np.matmul(mTa_homogeneous, drone_from_detected_homogeneous)

    # drone_in_map = transformation_functions.homogeneous2transform(drone_in_map_homogeneous)

    # mTd = PoseWithCovarianceStamped()

    # mTd.header = marker.header
    # mTd.header.frame_id = 'map'
    # mTd.pose.pose.position = drone_in_map.translation
    # mTd.pose.pose.orientation = drone_in_map.rotation
    # mTd.pose.covariance = measurement_covariance


    # Request the transform from Source: 'aruco/detectedX' to Target: 'map'
    dTm = tf_buf.lookup_transform('map', detected_from_drone.child_frame_id, marker.header.stamp, timeout=rospy.Duration(.2))
    
    # Find the old odom in map
    old_odom = tf_buf.lookup_transform('map', 'cf1/odom', marker.header.stamp, timeout=rospy.Duration(.2))
    
    odom_diff = transformation_functions.transform_diff(dTm.transform, mTa.transform)

    new_x = old_odom.transform.translation.x + odom_diff.translation.x
    new_y = old_odom.transform.translation.y + odom_diff.translation.y

    
    # Create the tranformation from 'map' to 'cf1/odom'
    odom = TransformStamped()
    odom.header.stamp = marker.header.stamp
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'cf1/odom_est'
    odom.transform.translation.x = new_x
    odom.transform.translation.y = new_y
    odom.transform.translation.z = 0.

    quaternion_diff = (odom_diff.rotation.x, odom_diff.rotation.y, odom_diff.rotation.z, odom_diff.rotation.w)
    quaternion_old = (old_odom.transform.rotation.x, old_odom.transform.rotation.y, old_odom.transform.rotation.z, old_odom.transform.rotation.w)

    euler_diff = euler_from_quaternion(quaternion_diff)
    euler_old = euler_from_quaternion(quaternion_old)

    yaw = euler_old[2] + euler_diff[2]

    quaternion = quaternion_from_euler(0., 0., yaw) 

    odom.transform.rotation.x = quaternion[0]
    odom.transform.rotation.y = quaternion[1]
    odom.transform.rotation.z = quaternion[2]
    odom.transform.rotation.w = quaternion[3]

    if not init_localisation:
        trans_broadcaster.sendTransform(odom)
        odom_measurement = TransformStamped()
        odom_measurement.header.stamp = marker.header.stamp
        odom_measurement.header.frame_id = odom.child_frame_id
        odom_measurement.child_frame_id = 'cf1/base_link_est'
        odom_measurement.transform.translation.y = drone_position.pose.position.y
        odom_measurement.transform.translation.x = drone_position.pose.position.x
        odom_measurement.transform.translation.z = drone_position.pose.position.z

        drone_euler = euler_from_quaternion((drone_position.pose.orientation.x, drone_position.pose.orientation.y, drone_position.pose.orientation.z, drone_position.pose.orientation.w))
        drone_yaw = drone_euler[2]
        drone_quaternion = quaternion_from_euler(0.,0.,drone_yaw)
        odom_measurement.transform.rotation.x = drone_quaternion[0]
        odom_measurement.transform.rotation.y = drone_quaternion[1]
        odom_measurement.transform.rotation.z = drone_quaternion[2]
        odom_measurement.transform.rotation.w = drone_quaternion[3]

        trans_broadcaster.sendTransform(odom_measurement)

        drone_in_map = tf_buf.lookup_transform('map', odom_measurement.child_frame_id, marker.header.stamp, rospy.Duration(.1))
        odom_updated.publish(drone_in_map)

    else:
        odom.child_frame_id = 'cf1/odom'
        trans_broadcaster.sendTransform(odom)
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
odom_updated = rospy.Publisher('/ekf/cf1_measurement', TransformStamped, queue_size=10)

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