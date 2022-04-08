#!/usr/bin/env python

import sys
import math
import json

import rospy
from tf.transformations import quaternion_from_euler
from localisation.msg import Landmark, LandmarkArray
from geometry_msgs.msg import Vector3

import unicodedata


def main(argv=sys.argv):
    # Let ROS filter through the arguments
    args = rospy.myargv(argv=argv)

    lma = LandmarkArray()

    # Load world JSON
    with open(args[1], 'rb') as f:
        world = json.load(f)

    lma.header.frame_id = 'map'
    lma.header.stamp = rospy.Time.now()

    if len(world['markers']) > 0:
        for marker in world['markers']:
            lm = Landmark()
            lm.header.frame_id = 'map'
            lm.header.stamp = rospy.Time.now()
            lm.id = marker['id']
            lm.name = 'ArucoMarker'
            lm.pose.position = Vector3(*marker['pose']['position'])
            roll, pitch, yaw = marker['pose']['orientation']
            (lm.pose.orientation.x,
             lm.pose.orientation.y,
             lm.pose.orientation.z,
             lm.pose.orientation.w) = quaternion_from_euler(math.radians(roll),
                                                            math.radians(pitch),
                                                            math.radians(yaw))
            lma.landmarks.append(lm)
            

    if len(world['roadsigns']) > 0:
        for marker in world['roadsigns']:
            lm = Landmark()
            lm.header.frame_id = 'map'
            lm.header.stamp = rospy.Time.now()
            lm.name = unicodedata.normalize('NFKD', marker['sign']).encode('ascii', 'ignore')
            lm.id = 0
            lm.pose.position = Vector3(*marker['pose']['position'])
            roll, pitch, yaw = marker['pose']['orientation']
            (lm.pose.orientation.x,
             lm.pose.orientation.y,
             lm.pose.orientation.z,
             lm.pose.orientation.w) = quaternion_from_euler(math.radians(roll),
                                                            math.radians(pitch),
                                                            math.radians(yaw))
            lma.landmarks.append(lm)

    pub.publish(lma)
    
    

    # Publish these transforms statically forever every 20 second




#rospy.spin()


if __name__ == "__main__":
    
    rospy.init_node('pub_landmarks')
    pub = rospy.Publisher('landmarks', LandmarkArray, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        main()
        rate.sleep()

        
