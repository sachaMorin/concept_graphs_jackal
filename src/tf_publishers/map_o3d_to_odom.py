#!/usr/bin/env python

import tf
import tf.transformations as ts
import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

def tf_to_matrix(tf):
    trans, rot = tf
    mx = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))
    return mx

def multiply_tfs(tf1, tf2):
    mx1 = tf_to_matrix(tf1)
    mx2 = tf_to_matrix(tf2)
    mx = mx1 @ mx2
    trans = ts.translation_from_matrix(mx)
    rot = ts.quaternion_from_matrix(mx)

    return trans, rot

if __name__ == '__main__':
    rospy.init_node('tf_map_o3d_to_odom')

    # Create a tf broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # Wait for the transforms to be available
    tf_listener = tf.TransformListener()


    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        try:
            # Look up the first transform
            transform1 = tf_listener.lookupTransform("map_o3d", "range_sensor_o3d", rospy.Time(0))

            # Look up the second transform
            transform2 = tf_listener.lookupTransform("velodyne", "odom", rospy.Time(0))

            trans, rot = multiply_tfs(transform1, transform2)

            # Broadcast the combined transform
            tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "odom", "map_o3d")
            # rospy.loginfo("Broadcasted transform")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to find transforms: %s", str(e))

        rate.sleep()

    rospy.spin()
