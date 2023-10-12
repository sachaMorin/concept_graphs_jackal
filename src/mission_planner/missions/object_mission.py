import rospy
import smach
import tf
import time

from .waypoint_mission import WaypointMission
from real_nav.srv import QueryService, DoISee


class ObjectMission(WaypointMission):
    def __init__(self, query, visual_mode):
        super().__init__(mission_data=dict(), reference_frame="map")
        self.query = query
        self.visual_mode = visual_mode
        
        self.tf_listener = tf.TransformListener()

        # Services
        self.query_goal = rospy.ServiceProxy("/concept_graph_tools/query_goal", QueryService)
        self.do_i_see = rospy.ServiceProxy("/concept_graph_tools/do_i_see", DoISee)

        # Look at objects
        self.look = True

        # Process main query
        rospy.loginfo(f"Received query '{self.query}'. ")
        rospy.loginfo("Querying GPT...")
        answer = self.query_goal(self.query, self.visual_mode, [])
        self.object_id_base, self.object_desc_base = answer.object_id, answer.object_desc
        rospy.loginfo(f"Found corresponding object described as '{self.object_desc_base}' with id {self.object_id_base}.")

        self.attempts = 0
        self.visited_objects = []

    def execute(self, userdata, loginfo=True):
        if self.attempts == 0:
            object_id, object_desc = self.object_id_base, self.object_desc_base
        else:
            # Process main query
            rospy.loginfo("Querying GPT...")

            # query = f"'The object described as '{self.object_desc_base}' is not in the scene. Find a likely container or storage space where someone is likely to have moved the object described as '{self.object_desc_base}'?"

            query = (f"The object described as '{self.object_desc_base}' is not in the scene. "
                         f"Perhaps someone has moved it, or put it away. "
                         f"Let's try to find the object by visiting the likely places, storage or containers that are "
                         f"appropriate for the missing object (eg: a a cabinet for a wineglass, or closet for a broom). "
                         f"So the new query is find a likely container or storage space where someone typically would"
                         f"have moved the object described as '{self.object_desc_base}'?")
            answer = self.query_goal(query, self.visual_mode, self.visited_objects)
            if not answer.query_achievable:
                rospy.logerr("GPT deemed the following query unachievable")
                rospy.logerr(query)
                return "Aborted"

            object_id, object_desc = answer.object_id, answer.object_desc

        if isinstance(object_id, int):
            self.visited_objects.append(object_id)
        else:
            rospy.logwarn(f"Ignoring object_id {object_id} in the visited objects buffer. Not an int.")

        # Get Object Position in map Frame
        rospy.loginfo(f"Attempting to reach {object_desc} with id {object_id}")

        user_input = input("Continue?")
        if user_input.lower() in ['q', 'exit']:
            exit()

        time.sleep(.5)
        trans, _ = self.tf_listener.lookupTransform("map", "object_location", rospy.Time(0))

        self.waypoint_idx = 0
        self.mission_data = {
           f"{object_desc}":  {"x_m": trans[0], "y_m": trans[1], "yaw_rad": 0.0 }
        }

        super().execute(userdata, loginfo=False)

        self.attempts += 1

        if self.next_waypoint:
            rospy.loginfo("'" + object_desc +
                          "' reached. Do I see " + self.object_desc_base + "?")
            time.sleep(1.)  # Make sure robot stopped moving

            object_in_image = self.do_i_see(self.object_desc_base).object_in_image

            if object_in_image:
                rospy.loginfo(f"Found {self.object_desc_base}. SUCCESS!")
                return "Completed"
            else:
                rospy.loginfo(f"Dit not find {self.object_desc_base}. Looking for likely location!")
                return 'Next Waypoint'
        else:
            rospy.logwarn(
                "Waypoint of mission unreachable. Aborting current mission.")
            return 'Aborted'
