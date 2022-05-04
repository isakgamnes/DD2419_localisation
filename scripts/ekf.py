#!/usr/bin/env python

from dis import dis
from glob import glob
import queue
import sys
import math
import json
from turtle import distance

from matplotlib.transforms import Transform
from genpy import Duration

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from localisation.msg import Landmark, LandmarkArray
from geometry_msgs.msg import Vector3

import numpy as np
from time import time

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import *
from geometry_msgs.msg import Vector3, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Bool

# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Extended Kalman Filter example (two-wheeled mobile robot)

# Supress scientific notation when printing NumPy arrays
np.set_printoptions(precision=3, suppress=True)

# A matrix
# 3x3 matrix -> number of states x number of states matrix
# Expresses how the state of the system [x,y,yaw] changes
# from k-1 to k when no control command is executed.
# Typically a robot on wheels only drives when the wheels are told to turn.
# For this case, A is the identity matrix.
# A is sometimes F in the literature.
A_k_minus_1 = np.array([[1.0,  0,   0],
                        [0, 1.0,   0],
                        [0,  0, 1.0]])

B = np.array([[1., 0., 0.],
              [0., 1., 0.],
              [0., 0., 1.]])

# Noise applied to the forward kinematics (calculation
# of the estimated state at time k from the state
# transition model of the mobile robot). This is a vector
# with the number of elements equal to the number of states
process_noise_v_k_minus_1 = np.array([0.02, 0.02, 0.06])

# State model noise covariance matrix Q_k
# When Q is large, the Kalman Filter tracks large changes in
# the sensor measurements more closely than for smaller Q.
# Q is a square matrix that has the same number of rows as states.
Q_k = np.array([[0.02,   0,   0],
                [0, 0.02,   0],
                [0,   0, 0.05]])

# Measurement matrix H_k
# Used to convert the predicted state estimate at time k
# into predicted sensor measurements at time k.
# In this case, H will be the identity matrix since the
# estimated state maps directly to state measurements from the
# odometry data [x, y, yaw]
# H has the same number of rows as sensor measurements
# and same number of columns as states.
H_k = np.array([[1.0,  0,   0],
                [0,  1.0, 0],
                [0,  0, 1.0]])

# Sensor measurement noise covariance matrix R_k
# Has the same number of rows and columns as sensor measurements.
# If we are sure about the measurements, R will be near zero.
R_k = np.array([[1,   0,    0],
                [0,   1,    0],
                [0,   0,  np.pi/4]])

# Sensor noise. This is a vector with the
# number of elements equal to the number of sensor measurements.
sensor_noise_w_k = np.array([0.05, 0.05, np.pi/8])

def ekf(z_k_observation_vector, state_estimate_k_minus_1,
        control_vector_k_minus_1, P_k_minus_1, B):
    global first_ekf_run, Q_k
    """
    Extended Kalman Filter. Fuses noisy sensor measurement to 
    create an optimal estimate of the state of the robotic system.

    INPUT
        :param z_k_observation_vector The observation from the Odometry
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].
        :param state_estimate_k_minus_1 The state estimate at time k-1
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].
        :param control_vector_k_minus_1 The control vector applied at time k-1
            3x1 NumPy Array [v,v,yaw rate] in the global reference frame
            in [meters per second,meters per second,radians per second].
        :param P_k_minus_1 The state covariance matrix estimate at time k-1
            3x3 NumPy Array
        :param B matrix converting the control input to state k

    OUTPUT
        :return state_estimate_k near-optimal state estimate at time k  
            3x1 NumPy Array ---> [meters,meters,radians]
        :return P_k state covariance_estimate for time k
            3x3 NumPy Array                 
    """
    if first_ekf_run:
        Q_k = np.array([[10.,   0,   0],
                        [0, 10.,   0],
                        [0,   0, 2*np.pi]])
        first_ekf_run = False
    else:
        Q_k = np.array([[0.02,   0,   0],
                        [0, 0.02,   0],
                        [0,   0, 0.01]])


    ######################### Predict #############################
    # Predict the state estimate at time k based on the state
    # estimate at time k-1 and the control input applied at time k-1.
    # rospy.logwarn_throttle(1,'B is: {}'.format(B))
    state_estimate_k =  np.matmul(A_k_minus_1, state_estimate_k_minus_1) + \
                        np.matmul(B, control_vector_k_minus_1) + \
                        (process_noise_v_k_minus_1)
    # rospy.logwarn_throttle(-1, 'Measurement: [{}, {}, {}]'.format(z_k_observation_vector[0],
    #                                                             z_k_observation_vector[1],
    #                                                             z_k_observation_vector[2]))
    # rospy.logerr_throttle(-1, 'State: [{}, {}, {}]'.format(state_estimate_k[0], state_estimate_k[1], state_estimate_k[2]))

    # rospy.logfatal_throttle(-1, 'Diff state and measurement: [{}, {}, {}]'.format(z_k_observation_vector[0]-state_estimate_k[0],
    #                                                                             z_k_observation_vector[1] - state_estimate_k[1],
    #                                                                             z_k_observation_vector[2]-state_estimate_k[2]))

    # rospy.logwarn_throttle(1,'State Estimate Before EKF={}'.format(state_estimate_k))

    # Predict the state covariance estimate based on the previous
    # covariance and some noise
    P_k = np.matmul(A_k_minus_1, np.matmul(P_k_minus_1, A_k_minus_1.T)) + Q_k

    ################### Update (Correct) ##########################
    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted
    # the sensor measurements would be for the current timestep k.
    measurement_residual_y_k = z_k_observation_vector - (
        (np.matmul(H_k, state_estimate_k)) + (
            sensor_noise_w_k))

    # rospy.logwarn_throttle(1,'Observation={}'.format(z_k_observation_vector))

    # Calculate the measurement residual covariance
    S_k = np.matmul(H_k, np.matmul(P_k, H_k.T)) + R_k

    # Calculate the near-optimal Kalman gain
    # We use pseudoinverse since some of the matrices might be
    # non-square or singular.
    K_k = np.matmul(P_k, np.matmul(H_k.T, np.linalg.pinv(S_k)))
    # rospy.logwarn_throttle(1, 'Kalman gain: {}'.format(K_k))

    # Calculate an updated state estimate for time k
    state_estimate_k = state_estimate_k + \
        np.matmul(K_k, measurement_residual_y_k)
    # rospy.logwarn_throttle(1, 'State={}'.format(z_k_observation_vector))
    # Update the state covariance estimate for time k
    P_k = P_k - np.matmul(K_k, np.matmul(H_k, P_k))

    # Return the updated state and covariance estimates
    return state_estimate_k, P_k

def update_drone_in_map(msg):
    global drone_in_map, state_estimate_k_minus_1, time_stamp_last_update, initialized

    drone_euler = euler_from_quaternion((msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w))
    # Initialise the drone location
    if not initialized:
        state_estimate_k_minus_1 = [msg.pose.position.x,
                                    msg.pose.position.y,
                                    drone_euler[2]]
        time_stamp_last_update = time()
        initialized = True

    # Update current position of drone in the map

    drone_in_map = [msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    drone_euler[0],
                    drone_euler[1],
                    drone_euler[2]]


def run_ekf(msg):
    global drone_in_map, P_k_minus_1, time_stamp_last_update, \
            state_estimate_k_minus_1, odom_drone_pose_k, \
            drone_in_map_k_minus_1, odom_drone_pose_k_minus_1

    measurement_euler = euler_from_quaternion((msg.transform.rotation.x,
                                              msg.transform.rotation.y,
                                              msg.transform.rotation.z,
                                              msg.transform.rotation.w))

    measurement_z_k = np.array([msg.transform.translation.x, 
                                msg.transform.translation.y, 
                                measurement_euler[2]])

    # Find the distance traveled since the last update
    delta_x = odom_drone_pose_k[0] - odom_drone_pose_k_minus_1[0]
    delta_y = odom_drone_pose_k[1] - odom_drone_pose_k_minus_1[1]
    delta_yaw = odom_drone_pose_k[2] - odom_drone_pose_k_minus_1[2]
    # rospy.logwarn_throttle(1, '{}, {}'.format(odom_drone_pose_k[0], odom_drone_pose_k_minus_1[0]))

    
    
    quat = quaternion_from_euler(0., 0., delta_yaw)

    distance_travelled = PoseStamped()
    distance_travelled.header = msg.header
    distance_travelled.pose.position.x = delta_x
    distance_travelled.pose.position.y = delta_y
    distance_travelled.pose.position.z = 0.
    distance_travelled.pose.orientation.x = quat[0]
    distance_travelled.pose.orientation.y = quat[1]
    distance_travelled.pose.orientation.z = quat[2]
    distance_travelled.pose.orientation.w = quat[3]

    deltas_in_map = tf_buf.transform(distance_travelled, 'map')

    delta_euler = euler_from_quaternion((deltas_in_map.pose.orientation.x,
                                        deltas_in_map.pose.orientation.y,
                                        deltas_in_map.pose.orientation.z,
                                        deltas_in_map.pose.orientation.w))

    control_vector_k_minus_1 = np.array([deltas_in_map.pose.position.x, deltas_in_map.pose.position.y, delta_euler[2]])
    #rospy.logwarn_throttle(1, 'Measurement: {}'.format(msg))
    state_estimate_k, P_k = ekf(
        measurement_z_k,  # Most recent sensor measurement
        state_estimate_k_minus_1,  # Our most recent estimate of the state
        control_vector_k_minus_1,  # Our most recent control input
        P_k_minus_1,  # Our most recent state covariance matrix
        B)  # Time interval

    time_stamp_last_update = time()

    P_k_minus_1 = P_k

    old_odom = tf_buf.lookup_transform('map', 'cf1/odom', msg.header.stamp, timeout=rospy.Duration(.2))

    old_odom_euler = euler_from_quaternion((old_odom.transform.rotation.x, old_odom.transform.rotation.y, old_odom.transform.rotation.z, old_odom.transform.rotation.w))
    # rospy.logwarn_throttle(-1, (state_estimate_k[0] - drone_in_map[0]))
    # rospy.logwarn_throttle(-1, (state_estimate_k[1] - drone_in_map[1]))
    updated_odom = TransformStamped()
    updated_odom.header = msg.header
    updated_odom.header.frame_id = 'map'
    updated_odom.child_frame_id = 'cf1/odom'
    updated_odom.transform.translation.x = old_odom.transform.translation.x + (state_estimate_k[0] - drone_in_map[0])
    updated_odom.transform.translation.y = old_odom.transform.translation.y + (state_estimate_k[1] - drone_in_map[1])
    updated_odom.transform.translation.z = 0.

    # rospy.logwarn_throttle(1, (state_estimate_k[2] - drone_in_map[5]))
    quaternion_odom = quaternion_from_euler(0., 0., old_odom_euler[2] + (state_estimate_k[2] - drone_in_map[5]))
    updated_odom.transform.rotation.x = quaternion_odom[0]
    updated_odom.transform.rotation.y = quaternion_odom[1]
    updated_odom.transform.rotation.z = quaternion_odom[2]
    updated_odom.transform.rotation.w = quaternion_odom[3]

    """if check_pose_in_airspace([state_estimate_k[0], state_estimate_k[1]]) \
    and check_pose_in_airspace([updated_odom.transform.rotation.x, updated_odom.transform.rotation.y]):
        odom_publisher.publish(updated_odom)
    else: 
        return"""
    odom_publisher.publish(updated_odom)
    odom_drone_pose_k_minus_1[0] = odom_drone_pose_k[0]
    odom_drone_pose_k_minus_1[1] = odom_drone_pose_k[1]
    odom_drone_pose_k_minus_1[2] = odom_drone_pose_k[2]
    state_estimate_k_minus_1[0] = state_estimate_k[0]
    state_estimate_k_minus_1[1] = state_estimate_k[1]
    state_estimate_k_minus_1[2] = state_estimate_k[2]

def check_pose_in_airspace(pose):
    global world_max, world_min
    in_x = pose[0] < world_max[0] and pose[0] > world_min[0]
    in_y = pose[1] < world_max[1] and pose[1] > world_min[1]
    return in_x and in_y
    

def update_odom_drone(msg):
    global odom_drone_pose_k
    odom_drone_pose_k[0] = msg.pose.position.x
    odom_drone_pose_k[1] = msg.pose.position.y
    euler = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
    odom_drone_pose_k[2] = euler[2]

if __name__ == "__main__":
    rospy.init_node('kalman_filter')
    rospy.logwarn('Initialising {}'.format(rospy.get_name()))

    with open(sys.argv[1], 'rb') as f:
        world = json.load(f)
        
    world_max = world['airspace']['max']
    world_min = world['airspace']['min']

    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    trans_broadcaster  = tf2_ros.TransformBroadcaster()

    pose_updater = rospy.Subscriber('ekf/cf1_pose', PoseStamped, callback=update_drone_in_map)
    odom_drone_updater = rospy.Subscriber(rospy.get_param('odom_baselink_topic'), PoseStamped, callback=update_odom_drone)
    pose_updater = rospy.Subscriber('ekf/cf1_measurement', TransformStamped, callback=run_ekf)
    odom_publisher = rospy.Publisher('/loc/odom_est', TransformStamped, queue_size=10)

    # Variables used for ros
    drone_in_map = [0., 0., 0., 0., 0., 0.]
    state_estimate_k_minus_1 = None
    odom_drone_pose_k = [0., 0., 0.]
    time_stamp_last_update = None
    odom_drone_pose_k_minus_1 = [0., 0., 0.]
    drone_in_map_k_minus_1 = [0., 0., 0.]
    P_k_minus_1 = np.array([[0.1,  0,   0],
                            [0,  0.1, 0],
                            [0,  0,   0.1]])
    initialized = False
    first_ekf_run = True

    while not rospy.is_shutdown():
        rospy.spin()
