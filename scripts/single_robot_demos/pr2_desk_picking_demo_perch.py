#! /usr/bin/env python
import sys
import math
import copy
import rospy
import actionlib
import tf
import numpy
from geometry_msgs.msg import Pose, PoseStamped
from sbpl_demos import perception_helpers
from sbpl_demos import perch_helpers
from sbpl_demos import grasping_helpers
from sbpl_demos import pr2_helpers
from sbpl_demos.perception_helpers import AR_TYPES

class Demo:
    def __init__(self):
#         self.STATIONARY = True
        self.STATIONARY = False
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.GripperCommand = pr2_helpers.GripperCommand()
        self.PointHead = pr2_helpers.PointHead()
        self.ARTagListener = perception_helpers.ARTagListener()
        self.PerchClient = perch_helpers.PerchClient()
#         self.PR2ARGrasping = grasping_helpers.PR2ARGrasping()
        self.MoveitMoveArm = pr2_helpers.MoveitMoveArm()
        self.TorsoCommand = pr2_helpers.TorsoCommand()
#         self.TuckArms = pr2_helpers.TuckArms()
        if(not self.STATIONARY):
            self.MoveBase = pr2_helpers.MoveBase()
        self.ArmJointCommand = pr2_helpers.ArmJointCommand()

        rospy.sleep(1.0)
        rospy.loginfo('All Action clients connected!')
        #rospy.loginfo('Commanding Untucking')
        #self.TuckArms.UntuckArms()
        self.PointHead.LookAt("base_footprint", 1.25, -0.4, 0)
        rospy.loginfo('Commanding torso')
        self.TorsoCommand.MoveTorso(0.2)

#     def computeObjectPoseRoutine(self, name, artype, ee_position, markers):
#         if artype is not AR_TYPES.GENERAL_OBJ:
#
#             obj_markers = self.ARTagListener.getMarkersByType(markers, artype)
#             cnt = 0
#             for obj in obj_markers.markers:
#                 valid_poses_candidates = self.PR2ARGrasping.getValidPosesByType(obj.pose.pose, artype)
#                 if(valid_poses_candidates):
#                     self.valid_poses.append(valid_poses_candidates)
#                     (best_candidate, best_distance) = self.PR2ARGrasping.getBestPoseAmongValid(valid_poses_candidates, ee_position)
#                     if(best_candidate):
#                         self.best_poses.append(best_candidate)
#                         self.best_distances.append(best_distance)
#                         self.best_poses_object.append(obj.pose.pose)
#
#                 # place a collision object at the marker
#                 obj.pose.header.stamp = rospy.Time.now()
#                 obj.pose.header.frame_id = "odom_combined"
#                 size = (0,0,0)
#                 if(artype == AR_TYPES.CUBOID_EDGE):
#                     size = (0.2, 0.05, 0.05)
#                 elif(artype == AR_TYPES.CYLINDER):
#                     size = (0.02, 0.02, 0.3)
#                 elif(artype == AR_TYPES.ROD_END):
#                     size = (0.07, 0.07, 0.15)
#                 elif(artype == AR_TYPES.CUBE):
#                     size = (0.07, 0.07, 0.07)
#                 self.MoveitMoveArm.AddCollisionObject("object_"+name+"_"+ str(cnt), obj.pose, size )
#                 rospy.sleep(2.0)
#                 cnt+=1
#
#         else:
#             print("AR_TYPE.GENERAL_OBJ is not currently supported!")
# #             grasp_pose_perch = self.PerchClient.getGraspPose(name)
# #             print grasp_pose_perch
# #             valid_poses_candidates = self.PR2ARGrasping.getValidPosesByType(grasp_pose_perch, AR_TYPES.ROD_END)
# #             if(valid_poses_candidates):
# #                 self.valid_poses.append(valid_poses_candidates)
# #                 (best_candidate, best_distance) = self.PR2ARGrasping.getBestPoseAmongValid(valid_poses_candidates, ee_position)
# #                 if(best_candidate):
# #                     self.best_poses.append(best_candidate)
# #                     self.best_distances.append(best_distance)
# #                     self.best_poses_object.append(obj.pose.pose)
# #                     print best_candidate


    def moveToWorkstationRoutine(self):
        if(not self.STATIONARY):
                #optionally move to alternate location
# HACK XXX
#                 rospy.loginfo('Tucking arms')
#                 self.TuckArms.TuckLeftArm()

# HACK XXX
#                 rospy.loginfo("Moving to wide pose")
#                 self.MoveitMoveArm.MoveRightToCarry()

                rospy.loginfo('Commanding base to Workstation')
                self.MoveBase.MoveToWorkstation()
# HACK XXX
#                 rospy.loginfo('Untucking arms')
#                 self.TuckArms.UntuckRightArms()

    def moveToInternDeskRoutine(self):
        if(not self.STATIONARY):
            rospy.loginfo('Commanding Tuckarms')
#             self.GripperCommand.Command('l', 0) #Close gripper
# HACK XXX
#             self.TuckArms.TuckArms()

            rospy.loginfo('Commanding base to intern desk...')
            self.MoveBase.MoveToInternDesk()

# HACK XXX
#             rospy.loginfo('Commanding Untucking')
#             self.TuckArms.UntuckRightArms()

#     def pickingRoutine(self):
#         self.valid_poses = [] # all IK-valid poses
#         self.best_poses = [] # the best poses, one per item
#         self.best_distances = [] # the best distances, one per item corresponding to best_poses
#         self.best_poses_object = [] #TODO should be a map between pose -> object (AR) pose
#
#         rospy.loginfo('Commanding right gripper open')
#         self.GripperCommand.Command('r', 1) #open grigger
#         ## AR TAG PICKING
#         # get current EE position
#         (ee_position, ee_quat) = self.tflistener.lookupTransform("odom_combined", "r_wrist_roll_link", rospy.Time())
#
#         #want to latch it at this time
#         (markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = self.ARTagListener.getMarkersAndCounts() 
#         rospy.loginfo(" Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
#             n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)
#
#         if(n_desks > 0):
#             rospy.loginfo("Inserting Desk Collision objects")
#             desk_markers = self.ARTagListener.getMarkersByType(markers, AR_TYPES.DESK)
#             cnt = 0
#             for desk in desk_markers.markers:
#                 desk.pose.header.frame_id = "odom_combined"
#                 desk.pose.header.stamp = rospy.Time.now()
#                 print desk
#                 self.MoveitMoveArm.AddDeskCollisionObject("desk_"+str(cnt), desk.pose)
#                 cnt+=1
#
#         if(n_cylinders >0):
#             rospy.loginfo("Computing poses for CYLINDERS")
#             self.computeObjectPoseRoutine("cylinder", AR_TYPES.CYLINDER, ee_position, markers)
#
#         if(n_cubes >0):
#             rospy.loginfo("Computing poses for CUBES")
#             self.computeObjectPoseRoutine("cubes", AR_TYPES.CUBE, ee_position, markers)
#
#         if(n_cuboid_edges >0):
#             rospy.loginfo("Computing poses for CUBOID_EDGE")
#             self.computeObjectPoseRoutine("cuboid_edge", AR_TYPES.CUBOID_EDGE, ee_position, markers)            
#
#         if(n_rod_ends >0):
#             rospy.loginfo("Computing poses for ROD_END")
#             self.computeObjectPoseRoutine("rod_end", AR_TYPES.ROD_END, ee_position, markers)

#     def pickingRoutinePerch(self, object_name):
#         rospy.loginfo('Commanding right gripper open')
#         self.GripperCommand.Command('r', 1) #open grigger
#
#         # get grasp and pre-grasp poses
#         (grasp_poses_perch, interp_poses_perch, distances_to_grasp) = self.PerchClient.getGraspPoses(object_name)
#         print grasp_poses_perch
#         print interp_poses_perch
#         print distances_to_grasp
#
#         # add collision model
#         (markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = self.ARTagListener.getMarkersAndCounts() 
#         rospy.loginfo(" Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
#             n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)
#         if(n_desks > 0):
#             rospy.loginfo("Inserting Desk Collision objects")
#             desk_markers = self.ARTagListener.getMarkersByType(markers, AR_TYPES.DESK)
#             cnt = 0
#             for desk_marker in desk_markers.markers:
#                 try:
#                     desk_marker.pose.header.frame_id = "odom_combined"
#                     #desk_marker.pose.header.stamp = rospy.Time.now()
#                     desk_marker_in_map = self.tflistener.transformPose("map", desk_marker.pose)
#                     print desk_marker_in_map
#                     self.MoveitMoveArm.AddDeskCollisionObject("desk_"+str(cnt), desk_marker_in_map)
#                     cnt+=1
#                 except (tf.LookupException):
#                     print "tf.LookupException: Desk collision model will not be added!"
#                     continue
#                 except (tf.ConnectivityException):
#                     print "tf.ConnectivityException"
#                     continue
#                 except (tf.ExtrapolationException):
#                     print "tf.ExtrapolationException"
#                     continue
#
#         return (grasp_poses_perch, interp_poses_perch, distances_to_grasp)

    def pickingRoutinePerchSpin(self):
        rospy.loginfo('Commanding right gripper open')
        self.GripperCommand.Command('r', 1) #open grigger

        # get grasp and pre-grasp poses
        rospy.logwarn("Waiting for user's object selection from tablet...")
        (grasp_poses_perch, interp_poses_perch, distances_to_grasp) = self.PerchClient.getGraspPosesSpin()
#         print grasp_poses_perch
#         print interp_poses_perch
#         print distances_to_grasp
        print("distances_to_grasp: ", distances_to_grasp)

        # add collision model
        (markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = self.ARTagListener.getMarkersAndCounts() 
        rospy.loginfo(" Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
            n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)
        if(n_desks > 0):
            rospy.loginfo("Inserting Desk Collision objects")
            desk_markers = self.ARTagListener.getMarkersByType(markers, AR_TYPES.DESK)
            cnt = 0
            for desk_marker in desk_markers.markers:
                try:
                    desk_marker.pose.header.frame_id = "odom_combined"
                    #desk_marker.pose.header.stamp = rospy.Time.now()
                    desk_marker_in_map = self.tflistener.transformPose("map", desk_marker.pose)
                    print "desk_marker_in_map"
                    print desk_marker_in_map
                    self.MoveitMoveArm.AddDeskCollisionObject("desk_"+str(cnt), desk_marker_in_map)
                    cnt+=1
                except (tf.LookupException):
                    print "tf.LookupException: Cannot find transform between /map and /odom_combined. Desk collision model will not be added!"
                    continue
                except (tf.ConnectivityException):
                    print "tf.ConnectivityException"
                    continue
                except (tf.ExtrapolationException):
                    print "tf.ExtrapolationException"
                    continue

        return (grasp_poses_perch, interp_poses_perch, distances_to_grasp)


    def compensateMoveItOdomToBaseError(self, odom_to_des_pose):

        # Compensate transformation error from /odom_combined to /base_footprint of MoveIt's Scene Robot
        # this error comes from 'planar' robot_collision_model/world_joint/type that virtually connects /odom_combined and /base_footprint

        # T_odom_to_desrev = T_odom_to_base * T_base_to_desrev
        #                  = T_odom_to_base * T_base_to_baserev * T_baserev_to_desrev
        #                  = T_odom_to_base * T_base_to_baserev * T_base_to_des
        #                  = T_odom_to_baserev * T_base_to_des

        # T_odom_to_baserev
        (odom_to_base_trans, odom_to_base_quat) = self.tflistener.lookupTransform("odom_combined", "base_footprint", rospy.Time())
        odom_to_baserev_trans = odom_to_base_trans
        odom_to_baserev_quat = (0.0, 0.0, odom_to_base_quat[2], odom_to_base_quat[3])   # planar world_joint constraint for quaternion/x,y to be zeros
        T_odom_to_baserev = self.tflistener.fromTranslationRotation(odom_to_baserev_trans, odom_to_baserev_quat)

        # base_to_des_pose
        odom_to_des_pose_stamped = PoseStamped()
        odom_to_des_pose_stamped.header.frame_id = "odom_combined"
        odom_to_des_pose_stamped.pose = odom_to_des_pose
        base_to_des_pose_stamped = self.tflistener.transformPose("base_footprint", odom_to_des_pose_stamped)
        base_to_des_pose = base_to_des_pose_stamped.pose

        # T_base_to_des
        base_to_des_pose = base_to_des_pose
        base_to_des_trans = (base_to_des_pose.position.x, base_to_des_pose.position.y, base_to_des_pose.position.z)
        base_to_des_quat = (base_to_des_pose.orientation.x, base_to_des_pose.orientation.y, base_to_des_pose.orientation.z, base_to_des_pose.orientation.w)
        T_base_to_des = self.tflistener.fromTranslationRotation(base_to_des_trans, base_to_des_quat)

        # T_odom_to_desrev
        T_odom_to_desrev = numpy.dot(T_odom_to_baserev, T_base_to_des)

        # convert transformation matrix to geometry_msgs/Pose
        odom_to_desrev_pose = self.PerchClient.convert_T_to_Pose(T_odom_to_desrev)

        return odom_to_desrev_pose


    def compensateKinectCalibrationError(self, odom_to_des_pose):

        # HACK hard-coded calibration offset
        offset_x_in_base = -0.005
        offset_y_in_base = -0.02
        (odom_to_base_trans, odom_to_base_quat) = self.tflistener.lookupTransform("odom_combined", "base_footprint", rospy.Time())
        T_odom_to_base = self.tflistener.fromTranslationRotation(odom_to_base_trans, odom_to_base_quat)

        odom_to_desrev_pose = copy.deepcopy(odom_to_des_pose)

        offset_x_in_odom = T_odom_to_base[:3,0] * offset_x_in_base
        offset_y_in_odom = T_odom_to_base[:3,1] * offset_y_in_base
        odom_to_desrev_pose.position.x += offset_x_in_odom[0] + offset_y_in_odom[0]
        odom_to_desrev_pose.position.y += offset_x_in_odom[1] + offset_y_in_odom[1]
        odom_to_desrev_pose.position.z += offset_x_in_odom[2] + offset_y_in_odom[2]

        return odom_to_desrev_pose


    def dropOffObjectRoutine(self, release_pose):
        rospy.loginfo("Moving to extend pose")
        self.MoveitMoveArm.MoveRightToExtend(release_pose)
        rospy.loginfo("Commanding right gripper open")
        self.GripperCommand.Command('r', 1) #open gripper
        self.MoveitMoveArm.MoveRightToShortExtend(release_pose)
# HACK XXX
#         rospy.loginfo('Commanding Tuckarms')
#         self.TuckArms.TuckLeftArm()
        rospy.loginfo("Moving to wide open")
        self.MoveitMoveArm.MoveRightToWide()

    def runDemo(self):
#         rospy.loginfo('Untucking arms')
#         self.TuckArms.UntuckRightArms()
#         self.moveToInternDeskRoutine()

#         rospy.loginfo("Moving to wide open")
#         self.MoveitMoveArm.MoveRightToWide()

        if(not self.STATIONARY):
            rospy.loginfo("Initialize PR2 pose")
            self.MoveBase.InitializePosePR2()

        rospy.loginfo("Moving arms to wide open")
        self.ArmJointCommand.MoveRightArmToWide()
        self.ArmJointCommand.MoveLeftArmToWide()

        while not rospy.is_shutdown():

#             self.pickingRoutine()
#
#             if not len(self.best_poses) > 0:
#                 rospy.logwarn("No poses found, retrying")
#                 self.MoveitMoveArm.removeAllObjectsAndDesks()
#                 rospy.sleep(1)
#                 continue
#
#             # select the best desired grasp among the candidates
#             best_index = min(xrange(len(self.best_distances)), key=self.best_distances.__getitem__)
#             grasp_pose = self.best_poses[best_index]
#             object_pose = self.best_poses_object[best_index]
#             interp_pose = self.PR2ARGrasping.getInterpolatedPose(grasp_pose, object_pose)
#
#             # correction for offset of interp_pose
#             factor_arm_x = -0.04
#             factor_arm_y = -0.01
#             (arm_trans, arm_quat) = self.tflistener.lookupTransform("odom_combined", "base_footprint", rospy.Time())
#             arm_matrix = self.tflistener.fromTranslationRotation(arm_trans, arm_quat)
#
#             arm_offset_x = arm_matrix[:3,0] * factor_arm_x
#             arm_offset_y = arm_matrix[:3,1] * factor_arm_y
#             interp_pose.position.x += arm_offset_x[0] + arm_offset_y[0]
#             interp_pose.position.y += arm_offset_x[1] + arm_offset_y[1]
#             interp_pose.position.z += arm_offset_x[2] + arm_offset_y[2]
#
#             # correction for offset of grasp_pose
#             grasp_pose.position.x += arm_offset_x[0] + arm_offset_y[0]
#             grasp_pose.position.y += arm_offset_x[1] + arm_offset_y[1]
#             grasp_pose.position.z += arm_offset_x[2] + arm_offset_y[2]
#
#             factor_wrist_x = 0.02
#             (wrist_trans, wrist_quat) = self.tflistener.lookupTransform("odom_combined", "r_wrist_roll_link", rospy.Time())
#             wrist_matrix = self.tflistener.fromTranslationRotation(wrist_trans, wrist_quat)
#
#             wrist_offset_x = wrist_matrix[:3,0] * factor_wrist_x
#             grasp_pose.position.x += wrist_offset_x[0] + 0.2
#             grasp_pose.position.y += wrist_offset_x[1] + 0.2 
#             grasp_pose.position.z += wrist_offset_x[2] + 0.2
#
#             import pdb; pdb.set_trace()
#             #just for visualization
#             self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
#                 (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
#                 rospy.Time.now(), "desired_grasp", "odom_combined")


            ## PERCH INSTEAD OF AR TAGS

#             object_name = "003_cracker_box"
#             (grasp_poses, interp_poses, distances_to_grasp) = self.pickingRoutinePerch(object_name)

            (grasp_poses, interp_poses, distances_to_grasp) = self.pickingRoutinePerchSpin()

            if not len(grasp_poses) > 0:
                rospy.logwarn("No poses found, retrying")
                self.MoveitMoveArm.removeAllObjectsAndDesks()
                rospy.sleep(1)
                continue

            # select the best desired grasp among the candidates
            best_index = min(xrange(len(distances_to_grasp)), key=distances_to_grasp.__getitem__)
            grasp_pose = grasp_poses[best_index]
            interp_pose = interp_poses[best_index]

            # just for visualization
            #self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
            #    (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
            #    rospy.Time.now(), "interp_pose_best", "odom_combined")
            #self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
            #    (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
            #    rospy.Time.now(), "grasp_pose_best", "odom_combined")


            # compensate transformation error from /odom_combined to /base_footprint of MoveIt's Scene Robot
            interp_pose = self.compensateMoveItOdomToBaseError(interp_pose)
            grasp_pose = self.compensateMoveItOdomToBaseError(grasp_pose)

            # just for visualization
            #self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
            #    (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
            #    rospy.Time.now(), "interp_pose_best_rev", "odom_combined")
            #self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
            #    (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
            #    rospy.Time.now(), "grasp_pose_best_rev", "odom_combined")


            # compensate Kinect calibration error
            #interp_pose = self.compensateKinectCalibrationError(interp_pose)
            #grasp_pose = self.compensateKinectCalibrationError(grasp_pose)

            # just for visualization
            self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
                (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
                rospy.Time.now(), "interp_pose_best_rev2", "odom_combined")
            self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
                (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
                rospy.Time.now(), "grasp_pose_best_rev2", "odom_combined")


            # Execute grasp plan
            rospy.loginfo("Moving to interpolated pose")
            success = self.MoveitMoveArm.MoveToPose(interp_pose, "odom_combined")
            if not success:
                rospy.logwarn("could not move to interpolated pose, retrying")
                self.MoveitMoveArm.MoveRightToWide()
                self.MoveitMoveArm.removeAllObjectsAndDesks()
                rospy.sleep(1)
                continue

            rospy.loginfo("Removing collision objects and moving to final grasp pose")
            self.MoveitMoveArm.removeAllObjects()
            success = self.MoveitMoveArm.MoveToPose(grasp_pose, "odom_combined")
            if not success:
                rospy.logwarn("could not move to interpolated pose, retrying")
                self.MoveitMoveArm.MoveRightToWide()
                self.MoveitMoveArm.removeAllObjectsAndDesks()
                rospy.sleep(1)
                continue

            # grip_success: True when completely closed, False when grasped something
#             grip_success = self.GripperCommand.Command('r', 0) #Close Gripper
#             if grip_success:
#                 rospy.loginfo("Failed to grasp. Going back to identifying object location.")
#                 self.MoveitMoveArm.MoveRightToWide()
#                 continue
#             rospy.loginfo("Succeeded to grasp.")
            grip_success = self.GripperCommand.Command('r', 0.55) #Close Gripper   # HACK for 003_cracker_box
            rospy.loginfo("GripperCommand returned %d, but assuming succeeded to grasp...", int(grip_success))

            # retract to interpolate pose
            rospy.loginfo("Moving back to interpolated pose")
            interp_pose.position.z += 0.15
            success = self.MoveitMoveArm.MoveToPose(interp_pose, "odom_combined")
            if not success:
                rospy.logwarn("could not move to interpolated pose, retrying")
                interp_pose.position.z -= 0.12
                success = self.MoveitMoveArm.MoveToPose(interp_pose, "odom_combined")
                if not success:
                    rospy.logwarn("could not move to interpolated pose, aborted")
                    self.MoveitMoveArm.MoveRightToWide()
                    self.MoveitMoveArm.removeAllObjectsAndDesks()
                    rospy.sleep(1)
                    continue

            rospy.loginfo("Moving to carry pose")
            self.MoveitMoveArm.MoveRightToWide()

            # compute release pose in base_footprint frame
            grasp_pose_stamped = PoseStamped()
            grasp_pose_stamped.header.frame_id = "odom_combined"
            grasp_pose_stamped.pose = grasp_pose
            release_pose_stamped = self.tflistener.transformPose("base_footprint", grasp_pose_stamped)
            release_pose = release_pose_stamped.pose

            # self.moveToWayPointRoutine()
            self.moveToWorkstationRoutine()
            #TODO clear desks, get AR poses for desk, and insert new Desk collision object

            # release the object
            self.dropOffObjectRoutine(release_pose)

#             self.GripperCommand.Command('r', 0) #Close Gripper
            self.MoveitMoveArm.removeAllObjectsAndDesks()

            self.moveToInternDeskRoutine()

        self.MoveitMoveArm.Cleanup()
        print('shutting down...')    


if __name__ == "__main__":
    rospy.init_node('pr2_grasp_test')
    demo = Demo()
    demo.runDemo()

