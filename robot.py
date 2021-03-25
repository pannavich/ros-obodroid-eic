#!/usr/bin/env python
"""
Obodroid Private Workshop 2021
Intro to Navigation Stack
Robot Node
- Get Signal from the referee and begin the Competition
- Sending move_base goal to Start
- Sending custom goal to custom behavior
- Stop the move_base goal

Author : Theppasith N. <theppasith.n@obodroid.com>
Date : 16-Mar-2021
"""
##############################################################################
# Imports
##############################################################################
import rospy
from std_msgs.msg import Empty, Bool
from modules.mission import RobotMission
from modules.move_base_interface import MoveBaseInterface


##############################################################################
# Class
##############################################################################


class Robot(object):
    """
    Robot Class
    """
    def __init__(self):
        # move_base communication module
        self.move_base_interface = MoveBaseInterface(self.move_base_finish)

        ################### CODE HERE ###################
        self.robot_mission = RobotMission()
        #################################################

        # Referee Signal Subscription
        self.start_signal_sub = rospy.Subscriber(
            '/start',
            Empty,
            self.on_referee_start_callback
        )

        self.finish_signal_sub = rospy.Subscriber(
            '/finish',
            Bool,
            self.on_referee_finish_callback
        )

        self.reset_signal_sub = rospy.Subscriber(
            '/reset',
            Empty,
            self.on_referee_reset_callback
        )

        # Prompt
        self.log("Initialization Completed ! - Waiting for Start Signal")

    def on_referee_start_callback(self, _):
        """
        On Start Signal Received !
        """
        self.log("Receiving START Signal !")
        #################################################
        ################### CODE HERE ###################
        #################################################
        # Setup the Mission Queue
        self.robot_mission.setup()
        # Pop the Frontmost Mission out
        mission, mission_type = self.robot_mission.get_mission()
        # Start That Mission
        self.do_mission(mission, mission_type)
        #################################################
        #################################################
        #################################################

    def on_referee_finish_callback(self, _):
        """
        On Finish Signal Received !
        - Referee will send finish signal to the contestants
        when robot is on the finish line
        - Robot must Stop after receiving the message
        """
        self.log("Receiving FINISH Signal !")
        self.move_base_interface.stop()

    def on_referee_reset_callback(self, _):
        """
        On Reset Signal Received !
        """
        pass

    def move_base_finish(self, result):
        """
        When Move Base Finish
        """
        self.log(result)
        #################################################
        ################### CODE HERE ###################
        #################################################
        # Mission is Success
        is_success = result is "SUCCEEDED"

        # Check if Misssion is Success
        if is_success:
            mission, mission_type = self.robot_mission.get_mission()
            # If we can Get Mission = Do Mission
            if mission:
                self.do_mission(mission, mission_type)
            else:
                # No Mission Left
                self.log("FINISH !! ")
                self.move_base_interface.stop()

        # Mission is not Success
        else:
            self.move_base_interface.stop()
        #################################################
        #################################################
        #################################################

    def do_mission(self, mission, mission_type):
        """
        Proxy to Send Mission to The place
        with corresponding type
        """
        self.log("Do Mission Type : {}".format(mission_type))

        if mission_type == "move_base":
            self.move_base_interface.send_goal(mission)
        elif mission_type == "custom":
            self.log("Custom Type Mission Received")
            self.log("Do Something")
        else:
            self.log(
                "mission_type[{}] is not defined in do_mission".format(
                    mission_type
                )
            )

    def log(self, msg):
        """
        Logging
        """
        rospy.loginfo(
            "[robot] : {}".format(msg)
        )

##############################################################################
# Main
##############################################################################
if __name__ == '__main__':
    rospy.init_node("robot_node")
    ROBOT = Robot()
    rospy.spin()
