#!/usr/bin/env python
"""
Obodroid Private Workshop 2021
Intro to Navigation Stack
Robot Node
- Move Base Interfacing Node

Author : Theppasith N. <theppasith.n@obodroid.com>
Date : 16-Mar-2021
"""
##############################################################################
# Imports
##############################################################################
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

##############################################################################
# Class
##############################################################################

class MoveBaseInterface(object):
    """
    Move Base Interfacing
    """
    def __init__(self, finish_callback):
        self.namespace = "move_base"
        self.finish_callback = finish_callback

        # Actionlib Client
        self.action_client = actionlib.SimpleActionClient(
            self.namespace,
            MoveBaseAction
        )

    def send_goal(self, goal):
        """
        Send goal to move_base
        """
        if goal is None:
            self.log("No Goal Defined")
            return

        self.log("Sending {}".format(goal))

        self.action_client.send_goal(
            goal,
            done_cb=self.finish_helper
        )

    def finish_helper(self, status, result):
        """
        Actionlib Finish Translation Helper
        """
        goal_status = [
            "PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED",
            "ABORTED", "REJECTED", "PREEMPTING", "RECALLING",
            "RECALLED", "LOST",
        ]
        self.log("Finish with STATUS : {}".format(goal_status[status]))

        # Sleep a little bit before sending next
        rospy.sleep(rospy.Duration(0.8))

        if goal_status[status] in ["SUCCEEDED", "ABORTED", "PREEMPTED"]:
            self.finish_callback(goal_status[status])

    def stop(self):
        """
        Stop Move Base
        """
        self.action_client.cancel_all_goals()

    def log(self, msg):
        """
        Logging
        """
        rospy.loginfo(
            "[move_base_interface] : {}".format(msg)
        )
