#! /usr/bin/env python
import sys
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, Quaternion
from sbpl_demos import pr2_helpers


class Demo:

    def __init__(self):
        self.tflistener = tf.TransformListener()
        self.MoveitMoveArm = pr2_helpers.MoveitMoveArm()
        rospy.sleep(1.0)
        rospy.loginfo('All Action clients connected!')


    def runDemo(self):

        test_poses = [
                Pose(position=Point(x=0.5, y=-0.3, z=0.8), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                Pose(position=Point(x=0.6, y=-0.2, z=0.8), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                Pose(position=Point(x=0.7, y=-0.1, z=0.8), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                Pose(position=Point(x=0.5, y=-0.3, z=0.8), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                ]


        for i in range(0, len(test_poses)):

            print("start planning and execution...")
            print(test_poses[i])

            success = self.MoveitMoveArm.MoveToPose(test_poses[i], "base_footprint")
            if not success:
                rospy.logwarn("Could not move to %d-th test pose!", i)
                continue

            print("done!")
            rospy.sleep(5)


        self.MoveitMoveArm.Cleanup()
        print('shutting down...')


if __name__ == "__main__":
    rospy.init_node('pr2_smpl_test')
    demo = Demo()
    demo.runDemo()

