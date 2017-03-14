#! /usr/bin/env python
import sys
import roslib
import rospy
import actionlib
import math
import copy

from geometry_msgs.msg import PointStamped, PoseStamped
from sbpl_demos.srv import PoseUpsampleRequest, PoseUpsample
from sbpl_demos.msg import XYZRPY
from sbpl_demos.perception_helpers import AR_TYPES

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

class ARGrasping:
    def __init__(self, planning_group, reference_frame, joint_names, robot_transform):
        print "waiting for pose_upsampling server..."
        rospy.wait_for_service('pose_upsampling')
        print ("Connected.")
        self.client = rospy.ServiceProxy('pose_upsampling', PoseUpsample)
        self.request = PoseUpsampleRequest()
        self.request.check_against_ik = True
        self.request.visualize_poses_with_tf = True
        self.request.planning_group = planning_group
        self.request.reference_frame = reference_frame
        self.request.joint_names = joint_names
        self.request.robot_transform_to_object = robot_transform

    def getValidPosesByType(self, input_pose, artype):
        self.request.object_pose = input_pose
        if(artype == AR_TYPES.CYLINDER):
            self.request.x_interval = [0]
            self.request.y_interval = [0]
            self.request.z_interval = [-0.1, -0.05]
            self.request.roll_interval = [0]
            self.request.pitch_interval = [0]
            self.request.yaw_interval = list(frange(-math.pi, math.pi, math.pi/10))
        if(artype == AR_TYPES.CUBE):
            self.request.x_interval = [0]
            self.request.y_interval = [0]
            self.request.z_interval = [-0.025]
            self.request.roll_interval = list(frange(-math.pi, math.pi, math.pi/2))
            self.request.pitch_interval = list(frange(-math.pi, math.pi, math.pi/2))
            self.request.yaw_interval = list(frange(-math.pi, math.pi, math.pi/2))
        if(artype == AR_TYPES.ROD_END):
            self.request.x_interval = [0]
            self.request.y_interval = [0]
            self.request.z_interval = [-0.05]
            self.request.roll_interval = [0]
            self.request.pitch_interval = [0]
            self.request.yaw_interval = list(frange(-math.pi, math.pi, math.pi/2))
        if(artype == AR_TYPES.CUBOID_FLAT):
            self.request.x_interval = [0]
            self.request.y_interval = [0]
            self.request.z_interval = [0]
            self.request.roll_interval = [0]
            self.request.pitch_interval = [0]
            self.request.yaw_interval = [0]
        if(artype == AR_TYPES.CUBOID_EDGE):
            self.request.x_interval = [-0.02, 0, 0.02]
            self.request.y_interval = [0]
            self.request.z_interval = [-0.05]
            self.request.roll_interval = [math.pi/2.0]
            self.request.pitch_interval = [-math.pi/2.0, math.pi/2.0]
            self.request.yaw_interval = [-math.pi/2.0]

        res = self.client(self.request)
        return res.valid_poses

    def getBestPoseByType(self, input_pose, artype, ee_position):
        valid_poses = self.getValidPosesByType(input_pose, artype)
        if(len(valid_poses) > 0):
            best_distance = 999999
            best_index = 0
            for i in range(0,len(valid_poses)):
                dx = ee_position[0] - valid_poses[i].position.x
                dy = ee_position[1] - valid_poses[i].position.y
                dz = ee_position[2] - valid_poses[i].position.z
                dist = math.sqrt(dx**2 + dy**2 + dz**2)
                if(dist < best_distance):
                    best_index = i
                    best_distance = dist
            return valid_poses[best_index], best_distance
        else:
            return False

    def getInterpolatedPose(self, grasp_pose, object_pose):
        dx = grasp_pose.position.x - object_pose.position.x
        dy = grasp_pose.position.y - object_pose.position.y
        dz = grasp_pose.position.z - object_pose.position.z
        mag = math.sqrt(dx**2 + dy**2 + dz**2)
        dist = 0.1 # offset distance meters
        interp_pose = copy.deepcopy(grasp_pose)
        interp_pose.position.x += dx * dist / mag
        interp_pose.position.y += dy * dist / mag
        interp_pose.position.z += dz * dist / mag
        return interp_pose


class RomanARGrasping(ARGrasping):
    def __init__(self):
        planning_group = "right_arm_and_torso"
        reference_frame = "map"
        joint_names = ["torso_joint1",
                       "limb_right_joint1",
                       "limb_right_joint2",
                       "limb_right_joint3",
                       "limb_right_joint4",
                       "limb_right_joint5",
                       "limb_right_joint6",
                       "limb_right_joint7"]
        robot_transform = XYZRPY()
        robot_transform.x = -0.3
        robot_transform.y = 0.0
        robot_transform.z = 0
        robot_transform.roll = math.pi/2.0
        robot_transform.pitch = 0.0
        robot_transform.yaw = 0
        ARGrasping.__init__(self, planning_group, reference_frame, joint_names, robot_transform)


class PR2ARGrasping(ARGrasping):
    def __init__(self):
        planning_group = "right_arm"
        reference_frame = "map"
        joint_names = ["r_elbow_flex_joint",
                       "r_forearm_roll_joint",
                       "r_shoulder_lift_joint", 
                       "r_shoulder_pan_joint",
                       "r_upper_arm_roll_joint",
                       "r_wrist_flex_joint",
                       "r_wrist_roll_joint"]
        robot_transform = XYZRPY()
        robot_transform.x = -0.18
        robot_transform.roll = 0.0
        robot_transform.pitch = 0.0
        robot_transform.yaw = 0
        ARGrasping.__init__(self, planning_group, reference_frame, joint_names, robot_transform)

