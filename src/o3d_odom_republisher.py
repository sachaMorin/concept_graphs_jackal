#!/usr/bin/env python
"""Republish the o3d pose estimate with a fixed diagonal covariance matrix."""

import numpy as np
import rospy
from nav_msgs.msg import Odometry

def odom_callback(odom_msg):
    # Modify the covariance values
    # new_covariance = [0.001] * 36  # Setting all elements to 0.05
    new_covariance = np.eye(6) * .25
    new_covariance[5, 5] = .25 # Yaw

    new_covariance = list(new_covariance.flatten())

    modified_odom = Odometry()
    modified_odom.header = odom_msg.header
    modified_odom.child_frame_id = odom_msg.child_frame_id
    modified_odom.pose = odom_msg.pose
    modified_odom.twist = odom_msg.twist
    modified_odom.pose.covariance = new_covariance
    modified_odom.twist.covariance = new_covariance

    # Publish the modified odometry message
    pub.publish(modified_odom)

if __name__ == '__main__':
    rospy.init_node('odometry_noise_republisher', anonymous=True)

    # Subscribe to the original odometry topic
    sub = rospy.Subscriber('/mapping_node/scan2map_odometry', Odometry, odom_callback)

    # Create a publisher for the modified odometry topic
    pub = rospy.Publisher('/mapping_node/scan2map_odometry_noise', Odometry, queue_size=10)

    rospy.spin()