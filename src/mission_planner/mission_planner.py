#!/usr/bin/env python

import argparse
import rospy
import yaml
import smach_ros
import smach
import os
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from missions.waypoint_mission import WaypointMission
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage
import time
import cv2

class MissionPlan():
    def __init__(self, missions_data, reference_frame):
        self.missions_data = missions_data
        self.reference_frame = reference_frame

    def createStateMachine(self):
        state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
        with state_machine:
            smach.StateMachine.add('Waypoint Mission', WaypointMission(self.missions_data['waypoint_mission'], self.reference_frame),
                                   transitions={'Completed': 'Success', 'Aborted': 'Failure', 'Next Waypoint': 'Waypoint Mission'})
        return state_machine

class MissionPlanner():
    def __init__(self, yaml_file_path, reference_frame):
        rospy.init_node('mission_planner_node')
        rospy.loginfo("Mission planner started.")
        # Read missions data.
        self.yaml_file_path = yaml_file_path
        self.reference_frame = reference_frame
        self.readMissionsData()

        # Subscribers
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw/compressed', CompressedImage, self.camera_callback,
                                           queue_size=6000)

        # Stuff to record the mission
        mission_id = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join("/home/administrator/cf_videos", mission_id)
        os.mkdir(path)
        # open(os.path.join(path, "llm.txt"), "a").close()
        open(os.path.join(path, "mission_planner.txt"), "a").close()
        self.pose_file = os.path.join(path, "poses.txt")
        with open(self.pose_file, "a") as f:
            f.write("#timestamp x y z qx qy qz qw\n")

        # Initialize tf listener
        self.tf_listener = tf.TransformListener()

        # Video buffer to save images
        # Image buffer
        height = 480
        width = 640
        self.image = np.zeros((height, width, 3), np.uint8)
        self.image_shape = (width, height)
        video_path = os.path.join(path, "front_camera.mp4")
        self._fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        self._video_out = cv2.VideoWriter(video_path, self._fourcc, 30.0, self.image_shape)

        self.main()

    def readMissionsData(self):
        with open(self.yaml_file_path, 'r') as file:
            self.missions_data = yaml.load(file, Loader=yaml.FullLoader)

    def main(self):
        rospy.init_node('mission_planner_node')
        rospy.loginfo("Mission planner started.")

        # Setup state machine.
        mission_plan = MissionPlan(self.missions_data, self.reference_frame)
        state_machine = mission_plan.createStateMachine()

        # Create and start the introspection server.
        introspection_server = smach_ros.IntrospectionServer('mission_planner_introspection_server', state_machine, '/mission_planner')
        introspection_server.start()

        # Execute state machine.
        outcome = state_machine.execute()
        rospy.loginfo("Mission plan terminated with outcome '" + outcome + "'.")

        # Wait for ctrl-c to stop the application
        introspection_server.stop()

    def camera_callback(self, data):
        bridge = CvBridge()

        try:
            # Save current pose
            time = rospy.Time(0)
            trans, rot = self.tf_listener.lookupTransform("map", "base_link", time)
            x, y, z = trans
            qx, qy, qz, qw = rot

            time = rospy.Time.now()
            with open(self.pose_file, "a") as f:
                f.write(
                    f"{str(time.secs)}.{str(time.nsecs)} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} \n")

            image = bridge.compressed_imgmsg_to_cv2(data)
            if hasattr(self, "_video_out"):
                self._video_out.write(image)
            self.image = image[:, :, ::-1]


        except CvBridgeError as e:
            print(e)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to find transforms: %s", str(e))
            return "Failed to send goal."

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Tool for planning missions.')
    parser.add_argument('mission_file_path', type=str, help='Path of the config file.')
    parser.add_argument('reference_frame', type=str, help='Name of the reference frame.')

    # Ignore arguments sent by roslaunch.
    parser.add_argument('__name:', help=argparse.SUPPRESS, nargs='?')
    parser.add_argument('__log:', help=argparse.SUPPRESS, nargs='?')
    args = parser.parse_args()

    mission_planner = MissionPlanner(args.mission_file_path, args.reference_frame)