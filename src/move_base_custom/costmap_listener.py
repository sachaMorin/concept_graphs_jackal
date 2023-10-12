#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from real_nav.srv import CheckCost


class CostmapListenerNode:
    def __init__(self):
        rospy.init_node('costmap_listener_node', anonymous=True)
        self.costmap = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None

        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.cost_service = rospy.Service('check_cost', CheckCost, self.check_cost_service)

    def costmap_callback(self, msg):
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def check_cost(self, x, y):
        if self.costmap is None:
            rospy.logwarn("Costmap not yet received")
            return None

        if x < self.map_origin_x or x > (self.map_origin_x + self.map_resolution * self.map_width):
            rospy.logwarn("X coordinate out of range")
            return None

        if y < self.map_origin_y or y > (self.map_origin_y + self.map_resolution * self.map_height):
            rospy.logwarn("Y coordinate out of range")
            return None

        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)

        cost = self.costmap[map_y, map_x]
        return cost

    def check_cost_service(self, msg):
        x = msg.x
        y = msg.y
        cost = self.check_cost(x, y)

        return {'cost': cost, 'success': cost is not None}


if __name__ == '__main__':
    try:
        node = CostmapListenerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
