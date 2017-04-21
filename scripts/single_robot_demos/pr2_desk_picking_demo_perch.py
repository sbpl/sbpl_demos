#! /usr/bin/env python
import sys
import math
import copy
import rospy
import actionlib
import tf
import numpy
import signal
from geometry_msgs.msg import Pose, PoseStamped
from sbpl_demos import perception_helpers
from sbpl_demos import perch_helpers
from sbpl_demos import grasping_helpers
from sbpl_demos import pr2_helpers
# from sbpl_demos.perception_helpers import AR_TYPES
from sbpl_demos.srv import StateMachine, StateMachineRequest

class Demo:
    def __init__(self):

#         self.STATIONARY = True
        self.STATIONARY = False

#         self.LARM_IN_USE = False    # manipulation with right arm
        self.LARM_IN_USE = True    # manipulation with left arm

        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.GripperCommand = pr2_helpers.GripperCommand(self.LARM_IN_USE)
        self.PointHead = pr2_helpers.PointHead()
#         self.ARTagListener = perception_helpers.ARTagListener()
        self.PerchClient = perch_helpers.PerchClient(self.LARM_IN_USE)
        self.MoveitMoveArm = pr2_helpers.MoveitMoveArm(self.LARM_IN_USE)
        self.TorsoCommand = pr2_helpers.TorsoCommand()
        if(not self.STATIONARY):
            self.MoveBase = pr2_helpers.MoveBase()
        self.ArmJointCommand = pr2_helpers.ArmJointCommand(self.LARM_IN_USE)

        print "waiting for state_machine server..."
        rospy.wait_for_service('state_machine')
        print ("Connected.")
        self.StateMachineClient = rospy.ServiceProxy('state_machine', StateMachine)
        self.StateMachineRequest = StateMachineRequest()

#         signal.signal(signal.SIGINT, self.signal_handler)
#         print('Press Ctrl+C')
#         signal.pause()

        rospy.sleep(1.0)
        rospy.loginfo('All Action clients connected!')
        rospy.loginfo("Move head to look forward")
        self.PointHead.LookAt("base_footprint", 1.25, 0.0, 0)


    def moveToWorkstationRoutine(self):

        ### UPDATE_PR2_STATE
        rospy.loginfo("Updating PR2's state!")
        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "PR2_STATE"
        self.StateMachineRequest.request_value = "MOVE_BASE"
        res = self.StateMachineClient(self.StateMachineRequest)
        rospy.loginfo("Updated 'PR2_STATE' on /state_machine!")

        if (not self.STATIONARY):
            rospy.loginfo('Commanding torso go up')
            self.TorsoCommand.MoveTorso(0.2)
            rospy.loginfo("Move head to look forward")
            self.PointHead.LookAt("base_footprint", 1.25, 0.0, 0)
            rospy.loginfo("Move to round table")
            res = self.MoveBase.MoveToWorkstation()
            rospy.loginfo('Commanding torso go down')
            self.TorsoCommand.MoveTorso(0.0)
            return res
        else:
            return True


    def moveToInternDeskRoutine(self):

        ### UPDATE_PR2_STATE
        rospy.loginfo("Updating PR2's state!")
        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "PR2_STATE"
        self.StateMachineRequest.request_value = "MOVE_BASE"
        res = self.StateMachineClient(self.StateMachineRequest)
        rospy.loginfo("Updated 'PR2_STATE' on /state_machine!")

        if (not self.STATIONARY):
            rospy.loginfo("Move head to look forward")
            self.PointHead.LookAt("base_footprint", 1.25, 0.0, 0)
            rospy.loginfo('Commanding torso go up')
            self.TorsoCommand.MoveTorso(0.2)
            rospy.loginfo("Move to intern desk")
            return self.MoveBase.MoveToInternDesk()
        else:
            return True


    def getGraspPosesPerch(self, object_name):
        # get grasp and pre-grasp poses
        (grasp_poses_perch, interp_poses_perch, distances_to_grasp) = self.PerchClient.getGraspPoses(object_name)
        #print grasp_poses_perch
        #print interp_poses_perch
        #print distances_to_grasp
        return (grasp_poses_perch, interp_poses_perch, distances_to_grasp)


    def getGraspPosesPerchSpin(self):
        # get grasp and pre-grasp poses
        rospy.logwarn("Waiting for user's object selection from tablet...")
        (grasp_poses_perch, interp_poses_perch, distances_to_grasp) = self.PerchClient.getGraspPosesSpin()
        #print grasp_poses_perch
        #print interp_poses_perch
        #print distances_to_grasp
        #print("distances_to_grasp: ", distances_to_grasp)
        return (grasp_poses_perch, interp_poses_perch, distances_to_grasp)


#     def addCollisionModels(self):
#         (markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = self.ARTagListener.getMarkersAndCounts() 
#         rospy.loginfo("Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
#             n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)
#         # collision model for intern desk
#         if(n_desks > 0):
#             rospy.loginfo("Inserting Desk Collision objects")
#             desk_markers = self.ARTagListener.getMarkersByType(markers, AR_TYPES.DESK)
#             cnt = 0
#             for desk_marker in desk_markers.markers:
#                 try:
#                     desk_marker.pose.header.frame_id = "odom_combined"
#                     #desk_marker.pose.header.stamp = rospy.Time.now()
#                     desk_marker_in_map = self.tflistener.transformPose("map", desk_marker.pose)
#                     #print "desk_marker_in_map"
#                     #print desk_marker_in_map
#                     self.MoveitMoveArm.AddDeskCollisionObject("desk_"+str(cnt), desk_marker_in_map)
#                     cnt+=1
#                 except (tf.LookupException):
#                     print "tf.LookupException: Cannot find transform between /map and /odom_combined. Desk collision model will not be added!"
#                     continue
#                 except (tf.ConnectivityException):
#                     print "tf.ConnectivityException"
#                     continue
#                 except (tf.ExtrapolationException):
#                     print "tf.ExtrapolationException"
#                     continue
#         # collision model for round table
#         if(n_cylinders > 0):
#             rospy.loginfo("Inserting Round Table Collision objects")
#             table_markers = self.ARTagListener.getMarkersByType(markers, AR_TYPES.CYLINDER)
#             cnt = 0
#             for table_marker in table_markers.markers:
#                 try:
#                     table_marker.pose.header.frame_id = "odom_combined"
#                     #table_marker.pose.header.stamp = rospy.Time.now()
# #                     table_marker_in_map = self.tflistener.transformPose("map", table_marker.pose)
#                     table_marker_in_map = self.tflistener.transformPose("odom_combined", table_marker.pose)
#                     #print "table_marker_in_map"
#                     #print table_marker_in_map
#                     self.MoveitMoveArm.AddTableCollisionObject("table_"+str(cnt), table_marker_in_map)
#                     cnt+=1
#                 except (tf.LookupException):
#                     print "tf.LookupException: Cannot find transform between /map and /odom_combined. Desk collision model will not be added!"
#                     continue
#                 except (tf.ConnectivityException):
#                     print "tf.ConnectivityException"
#                     continue
#                 except (tf.ExtrapolationException):
#                     print "tf.ExtrapolationException"
#                     continue
#         return True


    def addCollisionModelsInMap(self):
        if (not self.STATIONARY):
            self.MoveitMoveArm.AddDeskCollisionObjectInMap()
            self.MoveitMoveArm.AddTableCollisionObjectInMap()


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


    def selectGraspPose(self, grasp_poses, interp_poses, distances_to_grasp):

        # select the best desired grasp among the candidates
        best_index = min(xrange(len(distances_to_grasp)), key=distances_to_grasp.__getitem__)
        grasp_pose = grasp_poses[best_index]
        interp_pose = interp_poses[best_index]
        # for visualization
        #self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
        #    (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
        #    rospy.Time.now(), "interp_pose_best", "odom_combined")
        #self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
        #    (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
        #    rospy.Time.now(), "grasp_pose_best", "odom_combined")

        # compensate transformation error from /odom_combined to /base_footprint of MoveIt's Scene Robot
        interp_pose = self.compensateMoveItOdomToBaseError(interp_pose)
        grasp_pose = self.compensateMoveItOdomToBaseError(grasp_pose)
        # for visualization
        #self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
        #    (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
        #    rospy.Time.now(), "interp_pose_best_rev", "odom_combined")
        #self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
        #    (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
        #    rospy.Time.now(), "grasp_pose_best_rev", "odom_combined")

        # compensate Kinect calibration error
        #interp_pose = self.compensateKinectCalibrationError(interp_pose)
        #grasp_pose = self.compensateKinectCalibrationError(grasp_pose)
        # for visualization
        self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
            (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
            rospy.Time.now(), "interp_pose_best_rev2", "odom_combined")
        self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
            (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
            rospy.Time.now(), "grasp_pose_best_rev2", "odom_combined")

        # compute and save a release pose in base_footprint frame
        grasp_pose_stamped = PoseStamped()
        grasp_pose_stamped.header.frame_id = "odom_combined"
        grasp_pose_stamped.pose = grasp_pose
        release_pose_stamped = self.tflistener.transformPose("base_footprint", grasp_pose_stamped)
        self.release_pose = release_pose_stamped.pose

        return (grasp_pose, interp_pose)


    def graspObjectRoutine(self):

        ### UPDATE_PR2_STATE
        rospy.loginfo("Updating PR2's state!")
        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "PR2_STATE"
        self.StateMachineRequest.request_value = "MOVE_ARM"
        res = self.StateMachineClient(self.StateMachineRequest)
        rospy.loginfo("Updated 'PR2_STATE' on /state_machine!")


        # wait for a while to make Perch use the lastest/static observation
        rospy.loginfo('Commanding torso go down')
        self.TorsoCommand.MoveTorso(0.0)
        # look at the object
        rospy.loginfo("Move head to look left")
        self.PointHead.LookAt("base_footprint", 1.25, 0.5, 0)
        rospy.loginfo("20 sec left!")
        rospy.sleep(10)
        rospy.loginfo("10 sec left!")
        rospy.sleep(10)
        rospy.loginfo('Asking PERCH to detect the object')


        ### OPEN_GRIPPER
        rospy.loginfo('Commanding gripper open')
        self.GripperCommand.CommandGripperInUse(1) #open grigger


        ### DETECT_OBJECT
        # get candidate grasp poses
        # (grasp_poses, interp_poses, distances_to_grasp) = self.getGraspPosesPerchSpin()   # PR2-alone
        (grasp_poses, interp_poses, distances_to_grasp) = self.getGraspPosesPerch(self.PerchClient.getRequestedObjectName())     # PR2 + ROMAN
        if not len(grasp_poses) > 0:
            rospy.logwarn("No grasp poses found")
            self.MoveitMoveArm.removeAllObjectsAndDesks()
            return False
        # add collision models for detected AR markers
        self.addCollisionModelsInMap()


        ### SELECT_GRASP_POSE
        rospy.loginfo('Selecting the best grasp pose')
        (grasp_pose, interp_pose) = self.selectGraspPose(grasp_poses, interp_poses, distances_to_grasp)


        ### MOVE_ARM_TO_PREGRASP
        rospy.loginfo("Moving to pregrasp pose")
        success = self.MoveitMoveArm.MoveToPose(interp_pose, "odom_combined")
        if not success:
            rospy.logwarn("Could not move to pregrasp pose, now moving back to wide pose")
            success = self.MoveitMoveArm.MoveArmInUseToWide()
            if not success:
                rospy.logwarn("Could not move to wide pose, now do it forcefully!")
                self.ArmJointCommand.MoveArmInUseToWide()
            self.MoveitMoveArm.removeAllObjectsAndDesks()
            return False


        ### MOVE_ARM_TO_GRASP
        #rospy.loginfo("Removing collision objects and moving to final grasp pose")
        #self.MoveitMoveArm.removeAllObjects()   # CHECK is it safe to do this?
        rospy.loginfo("Moving to grasp pose")
        success = self.MoveitMoveArm.MoveToPose(grasp_pose, "odom_combined")
        if not success:
            rospy.logwarn("Could not move to grasp pose, now moving back to wide pose")
            success = self.MoveitMoveArm.MoveArmInUseToWide()
            if not success:
                rospy.logwarn("Could not move to wide pose, now do it forcefully!")
                self.ArmJointCommand.MoveArmInUseToWide()
            self.MoveitMoveArm.removeAllObjectsAndDesks()
            return False


        ### CLOSE_GRIPPER
        if self.PerchClient.getRequestedObjectName() == "003_cracker_box":
            rospy.loginfo('Commanding gripper close')
            grip_success = self.GripperCommand.CommandGripperInUse(0.55) #close Gripper   # HACK for 003_cracker_box
            rospy.loginfo("GripperCommand returned %d, but assuming succeeded to grasp...", int(grip_success))
        else:
            rospy.loginfo('Commanding gripper close')
            # grip_success: True when completely closed, False when grasped something
            grip_success = self.GripperCommand.CommandGripperInUse(0) #close Gripper
            if grip_success:
                rospy.loginfo("Failed to grasp, now moving back to wide pose")
                success = self.MoveitMoveArm.MoveArmInUseToWide()
                if not success:
                    rospy.logwarn("Could not move to wide pose, now do it forcefully!")
                    self.ArmJointCommand.MoveArmInUseToWide()
                self.MoveitMoveArm.removeAllObjectsAndDesks()
                return False
            rospy.loginfo("Succeeded to grasp.")


        ### MOVE_ARM_TO_PREGRASP
        rospy.loginfo("Moving back to pregrasp pose")
        interp_pose.position.z += 0.15
        success = self.MoveitMoveArm.MoveToPose(interp_pose, "odom_combined")
        if not success:
            rospy.logwarn("Could not move to pregrasp pose, retrying")
            interp_pose.position.z -= 0.12
            success = self.MoveitMoveArm.MoveToPose(interp_pose, "odom_combined")
            if not success:
                rospy.logwarn("Could not move to pregrasp pose, now moving back to wide pose")
                success = self.MoveitMoveArm.MoveArmInUseToWide()
                if not success:
                    rospy.logwarn("Could not move to wide pose, now do it forcefully!")
                    self.ArmJointCommand.MoveArmInUseToWide()
                    # NOTE we won't return False and just proceed to the next step


        ### MOVE_ARM_TO_WIDE
        rospy.loginfo("Moving back to wide pose")
        success = self.MoveitMoveArm.MoveArmInUseToWide()
        if not success:
            rospy.logwarn("Could not move to wide pose, now do it forcefully!")
            self.ArmJointCommand.MoveArmInUseToWide()
            # NOTE we won't return False and just proceed to the next step

        # remove all collision models
        self.MoveitMoveArm.removeAllObjectsAndDesks()

        return True


    def releaseObjectRoutine(self, release_pose):

        ### UPDATE_PR2_STATE
        rospy.loginfo("Updating PR2's state!")
        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "PR2_STATE"
        self.StateMachineRequest.request_value = "MOVE_ARM"
        res = self.StateMachineClient(self.StateMachineRequest)
        rospy.loginfo("Updated 'PR2_STATE' on /state_machine!")


        ### MOVE_ARM_TO_RELEASE
        self.addCollisionModelsInMap()
        rospy.sleep(1)
        rospy.loginfo("Moving to release pose")
        success_release = self.MoveitMoveArm.MoveArmInUseToExtend(release_pose)
        if not success_release:
#             rospy.logwarn("Could not move to release pose, now moving back to wide pose")
#             success = self.MoveitMoveArm.MoveArmInUseToWide()
#             if not success:
#                 rospy.logwarn("Could not move to wide pose, now do it forcefully!")
#                 self.ArmJointCommand.MoveArmInUseToWide()
#             self.MoveitMoveArm.removeAllObjectsAndDesks()
#
#             # escape from this state
#             rospy.logerr("I got stuck...")
#             rospy.logerr("Let me drop the object after 5 seconds!!!")
#             rospy.sleep(1)
#             rospy.logerr("Let me drop the object after 4 seconds!!!")
#             rospy.sleep(1)
#             rospy.logerr("Let me drop the object after 3 seconds!!!")
#             rospy.sleep(1)
#             rospy.logerr("Let me drop the object after 2 seconds!!!")
#             rospy.sleep(1)
#             rospy.logerr("Let me drop the object after 1 second!!!!")
#             rospy.sleep(1)
#             rospy.loginfo("Commanding gripper open")
#             self.GripperCommand.CommandGripperInUse(1) #open gripper
#             return False
            rospy.logwarn("Could not move to release pose, now move to pre-release pose forcefully!")
            self.ArmJointCommand.MoveArmInUseToPreRelease()
            rospy.sleep(10)
            rospy.logwarn("And now move to release pose forcefully!")
            self.ArmJointCommand.MoveArmInUseToPreRelease()
            rospy.sleep(5)
            # NOTE we won't return False and just proceed to the next step


        ### OPEN_GRIPPER
        rospy.loginfo("Commanding gripper open")
        self.GripperCommand.CommandGripperInUse(1) #open gripper


        ### MOVE_ARM_TO_POSTRELEASE
        if success_release:
            rospy.loginfo("Commanding gripper open")
            success = self.MoveitMoveArm.MoveArmInUseToShortExtend(release_pose)
            if not success:
                rospy.logwarn("Could not move to post-release pose, now moving back to wide pose")
                success = self.MoveitMoveArm.MoveArmInUseToWide()
                if not success:
                    rospy.logwarn("Could not move to wide pose, now do it forcefully!")
                    self.ArmJointCommand.MoveArmInUseToWide()
                    # NOTE we won't return False and just proceed to the next step
        else:
            rospy.logwarn("And now move to post-release pose forcefully!")
            self.ArmJointCommand.MoveArmInUseToPreRelease()
            rospy.sleep(5)


        ### MOVE_ARM_TO_WIDE
        rospy.loginfo("Moving to wide open")
        success = self.MoveitMoveArm.MoveArmInUseToWide()
        if not success:
            rospy.logwarn("Could not move to wide pose, now do it forcefully!")
            self.ArmJointCommand.MoveArmInUseToWide()
            # NOTE we won't return False and just proceed to the next step


        ### CLOSE_GRIPPER
        rospy.loginfo('Commanding gripper close')
        grip_success = self.GripperCommand.CommandGripperInUse(0) #close Gripper

        # remove all collision models
        self.MoveitMoveArm.removeAllObjectsAndDesks()

        return True


#     def signal_handler(signal, frame):
#         print('You pressed Ctrl+C!')
#         sys.exit(0)


    def runDemo(self):

        ### INITIALIZATION
        if(not self.STATIONARY):
            rospy.loginfo("Initialize PR2 pose")
            self.MoveBase.InitializePosePR2()
            rospy.sleep(1)

        ### MOVE_ARM_TO_WIDE + CLOSE_GRIPPER
        rospy.loginfo("Moving arms to wide open")
        self.ArmJointCommand.MoveArmInUseToWide()
        self.ArmJointCommand.MoveArmNotInUseToSide()
        self.GripperCommand.CommandGripperInUse(0) #close grigger
        self.GripperCommand.CommandGripperNotInUse(0) #close grigger

        ### WAIT_FOR_WEB
        rospy.loginfo("Waiting for user's object selection!")
        requested_object = self.PerchClient.getRequestedObjectNameSpin()
        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "requested_object"
        self.StateMachineRequest.request_value = requested_object
        res = self.StateMachineClient(self.StateMachineRequest)
        rospy.loginfo("Updated 'requested_object' on /state_machine!")

        ### WAIT_FOR_ROMAN
        rospy.loginfo("Waiting for Roman!")
        while not rospy.is_shutdown():
            self.StateMachineRequest.command = "Get"
            self.StateMachineRequest.request_key = "ROMAN_STATE"
            self.StateMachineRequest.request_value = ""
            res = self.StateMachineClient(self.StateMachineRequest)
            if res.result_value == "DONE":
                rospy.loginfo("Was told that Roman is done!")
                break
            else:
                rospy.sleep(1)


        #################### THE BEGINNING OF ONE CYCLE ####################

        ### MOVE_BASE_TO_TABLE
        initialized = True
        if (not self.moveToWorkstationRoutine()):
            rospy.logerr("Falied to moveToWorkstationRoutine()! Will retry to move to the round table once again after 5 seconds!")
            rospy.sleep(5)
            if (not self.moveToWorkstationRoutine()):
                rospy.logerr("Falied to moveToWorkstationRoutine() again! Exiting from the pipeline!")
                initialized = False

        while initialized and (not rospy.is_shutdown()):

            ### OPEN_GRIPPER + DETECT_OBJECT + SELECT_GRASP_POSE + MOVE_ARM_TO_PREGRASP + MOVE_ARM_TO_GRASP + CLOSE_GRIPPER + MOVE_ARM_TO_PREGRASP + MOVE_ARM_TO_WIDE
            if (not self.graspObjectRoutine()):
                rospy.logerr("Falied to graspObjectRoutine()! Restart from object detection again after 3 seconds!")
                rospy.sleep(3)
                continue

            ### MOVE_BASE_TO_DESK
            if (not self.moveToInternDeskRoutine()):
                rospy.logerr("Falied to moveToInternDeskRoutine()! Will retry to move to the intern desk once again after 5 seconds!")
                rospy.sleep(5)
                if (not self.moveToInternDeskRoutine()):
                    rospy.logerr("Falied to moveToInternDeskRoutine() again! Exiting from the pipeline!")
                    break

            ### MOVE_ARM_TO_RELEASE + OPEN_GRIPPER + MOVE_ARM_TO_POSTRELEASE + MOVE_ARM_TO_WIDE + CLOSE_GRIPPER
            if (not self.releaseObjectRoutine(self.release_pose)):
                rospy.logerr("Falied to releaseObjectRoutine()! Restart from moving to round table again after 3 seconds!")
                rospy.sleep(3)

            ### UPDATE_OBJECT_STATE
            rospy.loginfo("Resetting for user's object selection!")
            self.StateMachineRequest.command = "Set"
            self.StateMachineRequest.request_key = "requested_object"
            self.StateMachineRequest.request_value = ""
            res = self.StateMachineClient(self.StateMachineRequest)
            rospy.loginfo("Updated 'requested_object' on /state_machine!")

            ### UPDATE_PR2_STATE
            rospy.loginfo("Updating PR2's state!")
            self.StateMachineRequest.command = "Set"
            self.StateMachineRequest.request_key = "PR2_STATE"
            self.StateMachineRequest.request_value = "IDLE"
            res = self.StateMachineClient(self.StateMachineRequest)
            rospy.loginfo("Updated 'PR2_STATE' on /state_machine!")

        ####################### THE END OF ONE CYCLE #######################


            ### WAIT_FOR_WEB
            rospy.loginfo("Waiting for user's object selection!")
            requested_object = self.PerchClient.getRequestedObjectNameSpin()
            self.StateMachineRequest.command = "Set"
            self.StateMachineRequest.request_key = "requested_object"
            self.StateMachineRequest.request_value = requested_object
            res = self.StateMachineClient(self.StateMachineRequest)
            rospy.loginfo("Updated 'requested_object' on /state_machine!")

            ### WAIT_FOR_ROMAN
            rospy.loginfo("Waiting for Roman!")
            while not rospy.is_shutdown():
                self.StateMachineRequest.command = "Get"
                self.StateMachineRequest.request_key = "ROMAN_STATE"
                self.StateMachineRequest.request_value = ""
                res = self.StateMachineClient(self.StateMachineRequest)
                if res.result_value == "DONE":
                    rospy.loginfo("Was told that Roman is done!")
                    break
                else:
                    rospy.sleep(1)

            ### MOVE_BASE_TO_TABLE
            if (not self.moveToWorkstationRoutine()):
                rospy.logerr("Falied to moveToWorkstationRoutine()! Will retry to move to the round table once again after 5 seconds!")
                rospy.sleep(5)
                if (not self.moveToWorkstationRoutine()):
                    rospy.logerr("Falied to moveToWorkstationRoutine() again! Exiting from the pipeline!")
                    break


        ### DESTRUCTION
        self.MoveitMoveArm.Cleanup()

        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "requested_object"
        self.StateMachineRequest.request_value = ""
        res = self.StateMachineClient(self.StateMachineRequest)

        self.StateMachineRequest.command = "Set"
        self.StateMachineRequest.request_key = "ROMAN_STATE"
        self.StateMachineRequest.request_value = "IDLE"
        res = self.StateMachineClient(self.StateMachineRequest)

        print('Shutting down...')


if __name__ == "__main__":
    rospy.init_node('pr2_grasp_test')
    demo = Demo()
    demo.runDemo()

