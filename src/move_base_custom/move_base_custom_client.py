#!/usr/bin/env python3
# Move base wrapper with two additional options: 1) Go look at the goal pose and 2) rotate towards trajectory before starting

import numpy as np
import math
import tf
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from real_nav.srv import GoToObject, CheckCost
from nav_msgs.srv import GetPlan


class MoveBaseCustomClient:
    def __init__(self, node_name="MoveBaseCustomClient", as_node=True):
        super().__init__()

        if as_node:
            # Set this to false if you use the client in another ros node
            # Initialize Node
            rospy.init_node(node_name, anonymous=False)
            rospy.on_shutdown(self.on_shutdown)

        # Advertise current goal
        self.pose_pub = rospy.Publisher('custom_goal', PoseStamped, queue_size=10)

        # Initialize MoveBase client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for MoveBase server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("MoveBase server is available.")

        # Initialize Check Cost Service
        self.check_cost_service = rospy.ServiceProxy('check_cost', CheckCost)

        # Initialize Get Plan Service
        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        # Initialize tf listener
        self.tf_listener = tf.TransformListener()

    def find_looking_point(self, pose_msg, radius=1.75):
        x, y = pose_msg.pose.position.x, pose_msg.pose.position.y

        # Generate evenly spaced angles from 0 to 2*pi
        angles = np.linspace(0, 2 * np.pi, 24, endpoint=False)

        # Calculate x and y coordinates using parametric equations
        cos = np.cos(angles)
        sin = np.sin(angles)
        circle = np.array([cos, sin]).T
        circle_trans = circle * radius + np.array([x, y])

        result = []
        for (x_i, y_i) in circle_trans:
            cost = self.check_cost_service(x_i, y_i).cost
            result.append(cost)

        idx_min = np.argmin(result)
        x_goal, y_goal = circle_trans[idx_min]
        cos_goal, sin_goal = -circle[idx_min] # Flip vector to point towards the goal

        return self.get_pose_msg_vector(x_goal, y_goal, cos_goal, sin_goal)

    def get_pose_msg(self, x, y, z, w):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()  # Set the timestamp
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w

        return pose_msg

    def get_pose_msg_vector(self, x, y, delta_x, delta_y):
        # Convert a vector heading to a quaternion
        theta = math.atan2(delta_y, delta_x)  # Calculate the angle
        z = math.sin(theta / 2.0)  # Imaginary (vector) part z
        w = math.cos(theta / 2.0)  # Real (scalar) part

        return self.get_pose_msg(x, y, z, w)

    def get_pose_msg_yaw(self, x, y, yaw):
        z = math.sin(yaw / 2.0)  # Imaginary (vector) part z
        w = math.cos(yaw / 2.0)  # Real (scalar) part

        return self.get_pose_msg(x, y, z, w)

    def get_robot_pose(self):
        # Look for robot pose during one second
        self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
        trans, rot = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        return self.get_pose_msg(trans[0], trans[1], rot[2], rot[3])

    def _send_goal(self, pose_msg, done_cb=None, active_cb=None, feedback_cb=None):
        # First go to rotation goal
        goal = MoveBaseGoal(target_pose=pose_msg)
        self.move_base_client.send_goal(goal, done_cb, active_cb, feedback_cb)
        self.pose_pub.publish(pose_msg)

    def wait_for_result(self):
        self.move_base_client.wait_for_result()  # Wait for result

    def send_goal(self, goal_pose_msg, cb_args, done_cb=None, active_cb=None, feedback_cb=None):
        rotate, look = cb_args["rotate"], cb_args["look"]
        # rospy.logerr(f"ARGS rotate : {rotate} look : {look}")
        # rospy.logerr(f"Goal x : {goal_pose_msg.pose.position.x} y : {goal_pose_msg.pose.position.y}")
        if look:
            # Find a goal pose looking at the original goal
            goal_pose_msg = self.find_looking_point(goal_pose_msg)
            # rospy.logerr(f"After x : {goal_pose_msg.pose.position.x} y : {goal_pose_msg.pose.position.y}")

        if rotate:
            # Rotate towards trajectory before starting main execution
            robot_pose_msg = self.get_robot_pose()

            # Get the plan from current position to the goal
            plan = self.get_plan(robot_pose_msg, goal_pose_msg, 0.5).plan.poses

            # Look ahead 10 steps in the plan and calculate the angle
            look_ahead = min(10, len(plan) - 1)
            x, y = robot_pose_msg.pose.position.x, robot_pose_msg.pose.position.y
            delta_x = plan[look_ahead].pose.position.x - x
            delta_y = plan[look_ahead].pose.position.y - y
            rot_goal_pose_msg = self.get_pose_msg_vector(x, y, delta_x, delta_y)

            self._send_goal(rot_goal_pose_msg)
            self.wait_for_result()


        # Finally go to main goal
        self._send_goal(goal_pose_msg, done_cb, active_cb, feedback_cb)

    def spin(self):
        rospy.spin()

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    client = MoveBaseCustomClient()

    rot_pose_sub = rospy.Subscriber('/move_base_simple/rotate_goal', PoseStamped, client.send_goal,
                                    callback_args={'rotate': True, 'look': False})
    look_pose_sub = rospy.Subscriber('/move_base_simple/look_goal', PoseStamped, client.send_goal,
                                     callback_args={'rotate': True, 'look': True})

    client.spin()
