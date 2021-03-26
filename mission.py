#!/usr/bin/env python
"""
Obodroid Private Workshop 2021
Intro to Navigation Stack
Robot Node
- Robot Mission Module
- Arrange the mission being sent to move_base

Author : Theppasith N. <theppasith.n@obodroid.com>
Date : 16-Mar-2021
"""

##############################################################################
# Imports
##############################################################################
import rospy
from collections import deque
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

##############################################################################
# Mission
# (x,y,z,qx,qy,qz,qw,type)
##############################################################################
# WAYPOINTS = [
#     (-12.5404090881, 3.5620932579, 0, 0, 0, 0, 1, "move_base"),
#     (1.29557991028, 3.66241621971, 0, 0, 0, 0, 1, "move_base"),
#     (5.08437776566, -1.0588555336, 0, 0, 0, 0, 1, "move_base"),
#     (13.6356716156, 13.4348936081, 0, 0, 0, 0, 1, "move_base")
# ]


#world3
WAYPOINTS = [
    (-12.7313804626, 5.38211250305, 0, 0, 0, 0.704947491058, 0.70925949754, "move_base"),
    (-12.858,12.9406,0,0,0,0,1,"wait_obstacle"),
    (1.55,12.66,0,0,0,0,1,"move_base"),
    (13.28277,13.522688,0,0,0,0,1,"move_base")
]
##############################################################################
# Class
##############################################################################

class RobotMission(object):
    """
    Robot Mission Logic
    """
    def __init__(self):
        # Mission
        self.queue = deque()

    def setup(self):
        """
        Create new Mission Queue with Predefined Waypoints
        """
        self.queue.clear()

        # Fill the Queue
        for item in WAYPOINTS:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = item[0]
            goal.target_pose.pose.position.y = item[1]
            goal.target_pose.pose.position.z = item[2]
            goal.target_pose.pose.orientation.x = item[3]
            goal.target_pose.pose.orientation.y = item[4]
            goal.target_pose.pose.orientation.z = item[5]
            goal.target_pose.pose.orientation.w = item[6]

            # Append the Tuple (goal, goal_type)
            goal_type = item[7]
            mission = (goal, goal_type)

            self.queue.append(mission)

    def get_mission(self):
        """
        Start Sending Mission to Move_base
        """
        if len(self.queue) > 0:
            mission = self.queue.popleft()
            return mission
        else:
            return None, None

    def log(self, msg):
        """
        Logging
        """
        rospy.loginfo(
            "[mission] : {}".format(msg)
        )
