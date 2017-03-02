#!/usr/bin/env python

import sys
import math
import rospy
from sbpl_demos import pr2_helpers
from sbpl_demos import perception_helpers
import geometry_msgs
import tf
import copy
if __name__ == "__main__":
    rospy.init_node("pose_service_test")
    AR_listener = perception_helpers.ARTagListener()
    UpsampleGraspPoses = pr2_helpers.UpsampleGraspPoses()
    MoveitMoveArm = pr2_helpers.MoveitMoveArm()
    pr2_GripperCommand = pr2_helpers.GripperCommand()
    TuckArms = pr2_helpers.TuckArms()
    tfbroadcaster = tf.TransformBroadcaster()
    tflistener = tf.TransformListener()
    while(True):
        pr2_GripperCommand.Command('r', 1) #open gripper
        #while (2.0 < (rospy.Time.now() - AR_listener.last_reading).secs):
        #    #waiting for updated pose detection
        #    rospy.sleep(0.01)

        #print AR_listener.latest_pose
        
        valid_poses = UpsampleGraspPoses.getValidPosesForCylinder(AR_listener.latest_pose.pose)
        rospy.sleep(2.0)
        (ee_position, ee_quat) = tflistener.lookupTransform("map", "r_wrist_roll_link",rospy.Time())
        #print "ee_positions ", ee_position
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
            
        box_pose = AR_listener.latest_pose
        box_pose.pose.position.z -= .15
        MoveitMoveArm.moveit_planning_scene.add_box("cylinder", box_pose, size=(0.02, 0.02, 0.3) )
        i = best_index
        success = False
        dx = valid_poses[i].position.x - box_pose.pose.position.x
        dy = valid_poses[i].position.y - box_pose.pose.position.y
        dz = valid_poses[i].position.z - box_pose.pose.position.z
        mag = math.sqrt(dx**2 + dy**2 + dz**2)
        dist = 0.10
        interp_pose = copy.deepcopy(valid_poses[i])
        interp_pose.position.x += dx * dist / mag
        interp_pose.position.y += dy * dist / mag
        interp_pose.position.z += dz * dist / mag
        tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
            (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
            rospy.Time.now(), "desired_grasp", "map")
        print "Moving to interpolated pose"
        MoveitMoveArm.MoveToPose(interp_pose, "map")
        print "trying pose: " , i
        MoveitMoveArm.moveit_planning_scene.remove_world_object("cylinder")
        print "Moving to final pose"
        success = MoveitMoveArm.MoveToPose(valid_poses[i], "map")
        
        pr2_GripperCommand.Command('r', 0) #Close gripper
        TuckArms.TuckLeftArm()
        
        print len(valid_poses), " valid poses found!"
    #print resp