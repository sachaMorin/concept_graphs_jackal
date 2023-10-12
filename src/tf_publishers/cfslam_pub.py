#!/usr/bin/env python3
# Read the object x,y coordinates on /cf_object_location and latch the transform

import tf
import rospy
from std_msgs.msg import Float32MultiArray


class CFSLAMPublisher:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('CFSLAM_tf_pub', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize query publishers and subscribers
        self.query_sub = rospy.Subscriber('/cf_object_location', Float32MultiArray, self.object_callback)
        self.query = None

        # Intialize publisher
        self.tf_pub = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10)  # Tf Publisher frequency

        # Attributes to save current translation/quaternion
        self.trans = None
        self.rot = None


    def object_callback(self, msg):
        self.trans = msg.data
        self.rot = [0, 0, 0, 1]  # Quaternion

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.trans is not None:
                    self.tf_pub.sendTransform(self.trans, self.rot, rospy.Time.now(), "object_location",
                                                     "map_cf")
                    rospy.loginfo("Publishing transform!")
                else:
                    pass
                    # rospy.logwarn("No transform :(")
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down CFSLAM publisher.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = CFSLAMPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass