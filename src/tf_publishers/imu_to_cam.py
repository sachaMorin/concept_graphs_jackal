#!/usr/bin/env python

import tf
import rospy
import tf.transformations as ts

if __name__ == '__main__':
    rospy.init_node('tf_map_to_map_o3d')

    # Create a tf broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # From the extrinsics calibration (Thanks Miguel!)
    # mx = [[0.05763797, -0.9950165, -0.0813635, -0.01399428],
    #       [0.19503429, 0.09115129, -0.97655162, 0.10810955],
    #       [0.97910137, 0.04041778, 0.19931611, -0.04128748],
    #       [0., 0., 0., 1.]]
    mx = [[ 0.05763797,  0.19503429,  0.97910137,  0.02014616],
 [-0.9950165,   0.09115129,  0.04041778, -0.02211012],
 [-0.0813635,  -0.97655162,  0.19931611,  0.11266519],
 [ 0.,          0.,          0.,          1.,        ]]
    trans, rot = ts.translation_from_matrix(mx), ts.quaternion_from_matrix(mx)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            # Broadcast the combined transform
            tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "camera_link", "imu_link")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to find transforms: %s", str(e))

        rate.sleep()

    rospy.spin()
