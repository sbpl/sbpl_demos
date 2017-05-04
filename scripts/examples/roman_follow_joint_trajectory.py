#!/usr/bin/env python
import roslib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import actionlib
import actionlib
import rospy
import copy

class Arm:
    def __init__(self):
        #arm_name should be l_arm or r_arm
        self.jta = actionlib.SimpleActionClient('/right_limb/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

        self.jointstate = None
        self.jointstate_sub = rospy.Subscriber("/joint_states", JointState, self.__jointstatecallback__ )

    def __jointstatecallback__(self, jointstate):
        self.jointstate = jointstate

    def move(self, angles):
        goal = FollowJointTrajectoryGoal()

#        joint_goal.name = ['limb_right_joint1', 'limb_right_joint2', 'limb_right_joint3', 'limb_right_joint4', 'limb_right_joint5', 'limb_right_joint6', 'limb_right_joint7', 'torso_joint1']
#        joint_goal.position = [0.5661353269134606, 0.4692864095056042, -0.3136810029425956, -0.3174383113709873, -0.2514111373486396, 1.6025123702213988, 0.17182068670395123, 0.008224544748877419] # to fill in the values for a valid joint goal

        goal.trajectory.header.frame_id = 'map'
        goal.trajectory.joint_names = ['limb_right_joint1', 'limb_right_joint2', 'limb_right_joint3', 'limb_right_joint4', 'limb_right_joint5', 'limb_right_joint6', 'limb_right_joint7', 'torso_joint1']
        '''
        point1 = JointTrajectoryPoint()
        point1.positions(1) = self.jointstate(1)

        point1.positions = angles
        point1.time_from_start = rospy.Duration(5)
        goal.trajectory.points.append(copy.deepcopy(point1))

        point1.time_from_start = rospy.Duration(10)
        goal.trajectory.points.append(copy.deepcopy(point1))

        self.jta.send_goal_and_wait(goal)
        '''

        current = JointTrajectoryPoint()
        current.positions = list(self.jointstate.position[22:29])
        current.positions.append(self.jointstate.position[2])
        current.time_from_start = rospy.Duration(0)
        goal.trajectory.points.append(current)

        target = JointTrajectoryPoint()
        target.positions = angles
        target.time_from_start = rospy.Duration(5)
        goal.trajectory.points.append(target)

        self.jta.send_goal_and_wait(goal)

def main():
    arm = Arm()
    while arm.jointstate is None:
        doNothing = 1

    angles = [0.5661353269134606, 0.4692864095056042, -0.3136810029425956, -0.3174383113709873, -0.2514111373486396, 1.6025123702213988, 0.17182068670395123, 0.008224544748877419]
    #angles = [0.5661353269134606, 0.4692864095056042, -0.3136810029425956, -0.3174383113709873, -0.2514111373486396, 1.6025123702213988, 0.57182068670395123, 0.008224544748877419]
    #angles = [0.5661353269134606, 0.4692864095056042, -0.3136810029425956, -0.3174383113709873, -0.2514111373486396, 1.6025123702213988, 0.67182068670395123, 0.008224544748877419]
    arm.move(angles)

if __name__ == '__main__':
  rospy.init_node('joint_position_tester')
  main()

