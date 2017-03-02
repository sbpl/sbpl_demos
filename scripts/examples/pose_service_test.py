#!/usr/bin/env python

import sys
import rospy
from sbpl_demos.srv import PoseUpsampleRequest, PoseUpsample
from sbpl_demos.msg import XYZRPY
import geometry_msgs

if __name__ == "__main__":

    rospy.wait_for_service('pose_upsampling')
    client = rospy.ServiceProxy('pose_upsampling', PoseUpsample)
    request = PoseUpsampleRequest()
    request.check_against_ik = True
    request.visualize_poses_with_tf = True
    request.planning_group = "manipulator"
    request.reference_frame = "map"
    request.joint_names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    xyzrpy = XYZRPY()
    xyzrpy.x = 1
    xyzrpy.yaw = 1.57
    request.robot_transform_to_object = xyzrpy
    
    request.object_pose = geometry_msgs.msg.Pose()
    request.object_pose.orientation.w = 1
    request.roll_interval = [-3, -2, -1, 0, 1, 2, 3]
    request.yaw_interval = [-3, -2, -1, 0, 1, 2, 3]
    resp = client(request)
    print len(resp.valid_poses)
    #print resp