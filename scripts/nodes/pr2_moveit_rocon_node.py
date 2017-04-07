#! /usr/bin/env python
import math
import sys
import roslib
import rospy
import sbpl_demos.pr2_helpers as pr2
from sbpl_demos.msg import RoconMoveArmAction, RoconMoveArmGoal, RoconMoveArmResult
import actionlib

class PR2MoveitRocon(object):
    def __init__(self):
        self.MoveitMoveArm = pr2.MoveitMoveArm()
        self.server = actionlib.SimpleActionServer('rocon_move_arm', RoconMoveArmAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        success = self.MoveitMoveArm.MoveToPose(goal.target_pose)
        result = RoconMoveArmResult()
        result.success = success
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("pr2_moveit_rocon_node")
    srv = PR2MoveitRocon()
    rospy.spin()
    
