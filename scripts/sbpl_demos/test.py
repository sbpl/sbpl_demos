from sbpl_demos.roman_helpers import RomanCommandGripper, RomanMoveArm
import rospy

rospy.init_node("hello")
rma = RomanMoveArm()
rcg = RomanCommandGripper()

import IPython
IPython.embed()
