#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PoseStamped
from real_nav.srv import GoToObject, CheckCost

if __name__ == '__main__':
    # Read a transform and set it as a goal to move_base
    rospy.init_node('go_to_object')

    # Wait for the transforms to be available
    tf_listener = tf.TransformListener()
    
    pose_pub = rospy.Publisher('/move_base_simple/look_goal', PoseStamped, queue_size=10)


    def go_to_object(req):
        try:
            # Get Object Position
            trans, _ = tf_listener.lookupTransform("map", "object_location", rospy.Time(0))

            # Build target pose message
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = rospy.Time.now()  # Set the timestamp
            goal_pose_msg.header.frame_id = "map"
            goal_pose_msg.pose.position.x = trans[0]
            goal_pose_msg.pose.position.y = trans[1]
            goal_pose_msg.pose.position.z = 0.0
            goal_pose_msg.pose.orientation.x = 0.0
            goal_pose_msg.pose.orientation.y = 0.0
            goal_pose_msg.pose.orientation.z = 0.0
            goal_pose_msg.pose.orientation.w = 1.0

            pose_pub.publish(goal_pose_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to find transforms: %s", str(e))
            return "Failed to send goal."

        return f"Goal sent. Position : ({goal_pose_msg.pose.position.x:.3f}, {goal_pose_msg.pose.position.y:.3f}, {goal_pose_msg.pose.position.z:.3f}) Orientation : ({goal_pose_msg.pose.orientation.x:.3f}, {goal_pose_msg.pose.orientation.y:.3f}, {goal_pose_msg.pose.orientation.z:.3f}, {goal_pose_msg.pose.orientation.w:.3f})"

    service = rospy.Service("go_to_object", GoToObject, go_to_object)
    rospy.spin()