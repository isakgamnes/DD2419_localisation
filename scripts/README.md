# DD2419, localisation

This package contains the solution for localisation in the project course "Project course in Robotics and Autonomous Systems" offered at KTH.

## Node descriptions

### aruco_detection.py
This node will subscribe to the detected aruco markers published by the aruco package. It will then create a transform from the camera frame to the detected marker and publish this to the tf node.

### base_link_publisher.py
This node will subscribe to the "/cf1/pose" topic and publish a sequence of transformations from odom to base_link.
The sequence is: "/cf1/odom" -> "/cf1/baselink_footprint" -> "/cf1/baselink_stabilized" -> "/cf1/base_link"

### odom_publisher.py
This node will subscribes to "/loc/odom_est" which is an updated estimation of the position of the odom frame. It then updates its internal position and orientation of odom and continues to publish. The initial position of the odom is based on a random guess (0, 0, 0, 0, 0, 0, 1).

### odom_updater.py
This node subscribes to the detected aruco marker positions. These positions are then associated with one of the landmarks in the map, and then it calculates where in the map the odom frame has to be in order for the drone to receive that specific reading of the marker. The first marker is associated using the id of the aruco marker. After the first association has been done it will use euclidian distance to associate the markers.

### publish_landmarks.py
This node will read from the specified map file and publish all the landmarks that are in the map. The msg used is a custom made msg which contains all the data we find necessary and relevant for our solution. 

 

## Missing

The EKF node from the "robot_localisation" package

A node that calculates the position of the drone in the map based on any detected landmark. This position should be published to the EKF as a measurement input.

A node that calculates the position of the drone in the map based on the values from "/cf1/pose". This position should be published to the EKF as a control input.