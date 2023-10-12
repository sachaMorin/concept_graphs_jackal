#!/usr/bin/env python

import tf
import tf.transformations as ts
import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('tf_map_to_map_o3d')

    # Create a tf broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # Wait for the transforms to be available
    tf_listener = tf.TransformListener()


    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        try:
            trans, rot = tf_listener.lookupTransform("base_link", "velodyne", rospy.Time(0))

            # Broadcast the combined transform
            tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "map_o3d", "map")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to find transforms: %s", str(e))

        rate.sleep()

    rospy.spin()
