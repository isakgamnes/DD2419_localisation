#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Vector3
import tf2_ros 
from tf.transformations import *
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from std_msgs.msg import Bool
import time

def main():
    global odom_trans, is_localized,last_time_odom_was_uptdated


    if last_time_odom_was_uptdated < time.time() - 7:
        is_localized.data = False
        pub_localized.publish(is_localized)


    odom_trans.header.stamp = rospy.Time.now()

    trans_broadcaster.sendTransform(odom_trans)
    
    rate.sleep()


def updated_odom_callback(msg):
    global odom_trans, is_localized, last_time_odom_was_uptdated
    odom_trans = msg
    last_time_odom_was_uptdated = time.time()
    if not is_localized.data:
        is_localized.data = True
        pub_localized.publish(is_localized)
        rate.sleep()

if __name__ == "__main__":
    is_localized = Bool()
    is_localized.data = False
    # Make an initial guess on the odom position
    odom_trans = TransformStamped()
    # Set parent frame to 'map'
    odom_trans.header.frame_id = 'map'
    # Set child frame to 'odom'
    odom_trans.child_frame_id = 'cf1/odom'
    # Make the initial guess on pose and orientation
    pose = Vector3()
    pose.x = 0.
    pose.y = 0.
    pose.z = 0.
    odom_trans.transform.translation = pose
    orientation = Quaternion()
    orientation.x = 0.
    orientation.y = 0.
    orientation.z = 0.
    orientation.w = 1.
    odom_trans.transform.rotation = orientation

    
    rospy.init_node('odom_publisher')
    last_time_odom_was_uptdated = time.time()

    pub_localized = rospy.Publisher('/loc/is_localized', Bool, queue_size=10)
    
    odom_updater = rospy.Subscriber('/loc/odom_est', TransformStamped, callback=updated_odom_callback)
    tf_buf   = tf2_ros.Buffer()
    tf_lstn  = tf2_ros.TransformListener(tf_buf)
    trans_broadcaster  = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(20)
    pub_localized.publish(is_localized)
    while not rospy.is_shutdown():
        main()