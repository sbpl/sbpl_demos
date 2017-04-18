#! /usr/bin/env python
import rospy
from sbpl_demos import pr2_helpers

if __name__ == "__main__":
    rospy.init_node('pr2_grasp_test')
    TuckArms = pr2_helpers.TuckArms()
    TuckArms.TuckArms()
