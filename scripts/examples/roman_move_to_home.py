#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
import math
from tf.transformations import quaternion_from_euler
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
import sbpl_demos.grasping_helpers as grasping_helpers
import sbpl_demos.perception_helpers as perception_helpers
from sbpl_demos.perception_helpers import AR_TYPES
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
from sbpl_demos.perch_helpers import PerchClient


class RomanMove:
    def __init__(self):
        self.perch = PerchClient()
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.RMAC = RomanMoveArm() 
        self.RCG = RomanCommandGripper()
       # 

	#self.addtable.addCylinder('roundtable',0.3,0.5,0.468,-0.767,0.3)
        #self.RomanARGrasping = grasping_helpers.RomanARGrasping()
        #self.ARTagListener = perception_helpers.ARTagListener()

    

    def moveToGrasp(self):
        pose = Pose()
        #pose.position.x = 0.8601
        #pose.position.y = -0.304
        #pose.position.z = 0.24126
        #pose.orientation.x = 0.424
        #pose.orientation.y = 0.48864
        #pose.orientation.z = 0.34858
        #pose.orientation.w = 0.6782
        """pose.position.x = 0.9174
        pose.position.y = -0.2263
        pose.position.z = 0.4823
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
	self.RMAC.MoveToPose(pose, "base_footprint")"""
	pose.position.x = 0.568
        pose.position.y = -0.667
        pose.position.z = 1.070
        quat = quaternion_from_euler(-math.pi/2,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.RMAC.MoveFromPre(pose, "base_footprint")

    def moveToPreGrasp(self):
        pose = Pose()
        #pose.position.x = 0.8042
        #pose.position.y = -0.4814
        #pose.position.z = 0.3147
        #pose.orientation.x = 0.424
        #pose.orientation.y = 0.48864
        #pose.orientation.z = 0.34858
        #pose.orientation.w = 0.6782
        pose.position.x = 0.8077
        pose.position.y = -0.3721
        #pose.position.z = 0.5642
	pose.position.z = 1.7901
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
        self.RMAC.MoveToPose(pose, "base_footprint")

    def moveToTable(self):
        pose = Pose()
        """pose.position.x = 0.45613
        pose.position.y = -0.99926
        pose.position.z = 0.9072
        pose.orientation.w = 0.596323
        pose.orientation.x = 0.608277
        pose.orientation.y = -0.230372
        pose.orientation.z = -0.470453"""
	pose.position.x = 0.45613
        pose.position.y = -0.99926
        pose.position.z = 0.9072
        pose.orientation.w = 0.596323
        pose.orientation.x = 0.608277
        pose.orientation.y = -0.230372
        pose.orientation.z = -0.470453
        self.RMAC.MoveFromPre(pose, "base_footprint")
 
    def moveToPreTable(self):
        pose = Pose()
        pose.position.x = 0.36589
        pose.position.y = -0.830991
        #pose.position.z = 0.96672
	pose.position.z = 1.96672

        pose.orientation.w = 0.596323
        pose.orientation.x = 0.608277
        pose.orientation.y = -0.23037
        pose.orientation.z = -0.47045
        self.RMAC.MoveToPose(pose, "base_footprint")

def copysign(val, thing):
    if thing < 0:
        return -val
    else:
        return val;

def unravel_roman_joint_values(current_state, target_state):
    '''
    current_state and seed_state are sequences of 8 doubles for joints variables
    in the order 'limb_right_joint1', 'limb_right_joint2', 'limb_right_joint3',
    'limb_right_joint4', 'limb_right_joint5', 'limb_right_joint6',
    'limb_right_joint7', 'torso_joint1'.

    Returns a sequence of joint variables equivalent to the target state such that
    the resulting values are the nearest 2*pi equivalent to the current state.

    The torso joint is ignored.
    '''
    min_limits = [
        -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi,
        -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi, -2.0 * math.pi,
    ]

    max_limits = [
        2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi,
        2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi,
    ]

    close_joint_values = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    for i in range(len(current_state)):
        if i == 7: # skip the torso joint
            close_joint_values[i] = target_state[i]
            continue

        spos = target_state[i]
        vdiff = current_state[i] - spos
        twopi_hops = int(abs(vdiff / (2.0 * math.pi)))

        print 'twopi_hops: {0}'.format(twopi_hops)

        npos = spos + 2.0 * math.pi * twopi_hops * copysign(1.0, vdiff)
        if abs(npos - current_state[i]) > math.pi:
            npos = npos + 2.0 * math.pi * copysign(1.0, vdiff)

        if npos < min_limits[i] or npos > max_limits[i]:
            npos = spos

        close_joint_values[i] = npos

    return close_joint_values

def extract_joint_variables(joint_state, joint_names):
    '''Extract the variables for a subset of joints from a JointState.
    '''
    jvals = {}
    for i in range(len(joint_state.name)):
        jvals[joint_state.name[i]] = joint_state.position[i]

    joint_values = []
    for joint_name in joint_names:
       joint_values.append(jvals[joint_name])

    return joint_values

if __name__ == "__main__":
    rospy.init_node('roman_move_to_home')
    test = RomanMove()
    #test.RMAC.addcollisiontable()

    testGoal = Pose()
    testGoal.position.x = 1.008
    testGoal.position.y = -0.418 + 0.15
    testGoal.position.z = 1.171 #+ 0.15
    testGoal.orientation.x = -0.641
    testGoal.orientation.y = -0.297
    testGoal.orientation.z = 0.292
    testGoal.orientation.w = 0.645
    test.RMAC.MoveToPoseBlind(testGoal, 'base_footprint')

#    testGoal = PoseStamped()
#    testGoal.pose.position.x = 0.45613
#    testGoal.pose.position.y = -0.99926
#    testGoal.pose.position.z = 0.9072
#    testGoal.pose.orientation.w = 0.596323
#    testGoal.pose.orientation.x = 0.608277
#    testGoal.pose.orientation.y = -0.230372
#    testGoal.pose.orientation.z = -0.470453
#    testGoal.header.frame_id = "base_footprint"
#    testGoal.header.stamp = rospy.Time.now()
#    sol = test.RMAC.GenerateIK(testGoal.pose, reference_frame="base_footprint")
#
#    joint_names = [
#        'limb_right_joint1', 'limb_right_joint2', 'limb_right_joint3',
#        'limb_right_joint4', 'limb_right_joint5', 'limb_right_joint6',
#        'limb_right_joint7', 'torso_joint1'
#    ]
#    current_joint_values = extract_joint_variables(test.RMAC.jointstate, joint_names)
#    target_joint_values = extract_joint_variables(sol.solution.joint_state, joint_names)
#
#    print current_joint_values
#    print target_joint_values
#    print unravel_roman_joint_values(current_joint_values, target_joint_values)
   
    #test.RMAC.MoveToHome()
    #test.RMAC.moveToHome_jointgoal()
    #test.moveToGrasp()
    #test.RCG.Open()
    #test.moveToPreGrasp()
    #test.moveToGrasp()
    #test.RCG.Close()
    #test.moveToPreGrasp()
    #test.moveToPreTable()
    #test.moveToTable()
    #test.RCG.Open()
    #test.moveToPreTable()
    #test.RMAC.MoveToHome()"""
