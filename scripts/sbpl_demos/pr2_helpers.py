#! /usr/bin/env python
import sys
import roslib
import rospy
import actionlib
import math

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, PoseStamped
from sbpl_demos.msg import RoconMoveArmAction, RoconMoveArmGoal, RoconMoveArmResult
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal
from sbpl_demos.srv import PoseUpsampleRequest, PoseUpsample
from sbpl_demos.msg import XYZRPY
from groovy_pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from groovy_trajectory_msgs.msg import JointTrajectoryPoint

## for moveit commander
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import tf

from sbpl_demos import octomap_helpers


class TFLookup:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('tf2_buffer_server', LookupTransformAction)
        rospy.loginfo("waiting for tf2_buffer_server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")

    def getTransform(self, source_frame, target_frame):
        goal = LookupTransformGoal()
        goal.target_frame = target_frame
        goal.source_frame = source_frame
        goal.timeout = rospy.Time(1)
        self.client.send_goal(goal)
        if self.client.wait_for_result():
            res = self.client.get_result()
            return res.transform
        else:
            raise ValueError('tf2_buffer_server was not called correctly')
            return LookupTransformResult()

class PointHead:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        rospy.loginfo("Waiting for point_head_action...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")

    def LookAt(self, frame, x, y, z):
        goal=PointHeadGoal()
        point=PointStamped()
        point.point.x=x
        point.point.y=y
        point.point.z=z
        point.header.frame_id=frame
        
        goal.target=point
        goal.pointing_frame="high_def_frame"
        goal.min_duration=rospy.Duration(0.5)
        goal.max_velocity=1.0

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(3.0)):
            return True 
        else:
            return False

class TorsoCommand:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/torso_controller/position_joint_action', SingleJointPositionAction)
        rospy.loginfo("waiting for server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")
    def MoveTorso(self, height):
        goal = SingleJointPositionGoal()
        goal.position = height 
        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(3.0)):
            return True
        else:
            return False

class TuckArms:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/tuck_arms", TuckArmsAction)
        rospy.loginfo("Waiting for tuck arms server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")
    def TuckArms(self):
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30.0))

    def TuckLeftArm(self):
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = False
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30.0))

    def TuckRightArm(self):
        goal = TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = True
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30.0))

    def UntuckArms(self):
        goal = TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = False
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30.0))

    def UntuckRightArms(self):
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = False
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(30.0))

class GripperCommand:
    def __init__(self, LARM_IN_USE):

        self.LARM_IN_USE = LARM_IN_USE    # True: manipulation with left arm, False: with right arm

        self.left_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', GripperCommandAction)
        self.right_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', GripperCommandAction)
        rospy.loginfo("waiting for gripper actions...")
        self.left_client.wait_for_server()
        self.right_client.wait_for_server()
        rospy.loginfo("Connected.")

    def Command(self, gripper, open):

        goal = GripperCommandGoal()

        # 1) open/close only
        # if open == 1:
        #     goal.command.position = 0.09
        # else:
        #     goal.command.position = 0.0

        # 2) continuous value command
        if open > 1.0:
            rospy.logwarn("GripperCommand.Command() only accepts value from 0 (close) to 1 (open)")
            open = 1.0
        elif open < 0.0:
            rospy.logwarn("GripperCommand.Command() only accepts value from 0 (close) to 1 (open)")
            open = 0.0
        goal.command.position = (0.09 - 0.0) * open/(1.0 - 0.0) + 0.0

        goal.command.max_effort = float(-1.0) 
        
        if gripper == 'l':
            self.left_client.send_goal(goal)
            if self.left_client.wait_for_result(rospy.Duration(3.0)):
                res = self.left_client.get_result()
                if res.reached_goal:
                    return True
                return False
            else:
                return False
        else:
            self.right_client.send_goal(goal)
            if self.right_client.wait_for_result(rospy.Duration(3.0)):
                res = self.right_client.get_result()
                if res.reached_goal:
                    return True
                return False
            else:
                return False

    def CommandGripperInUse(self, open):
        if self.LARM_IN_USE:
            return self.Command('l',open)
        else:
            return self.Command('r',open)

    def CommandGripperNotInUse(self, open):
        if self.LARM_IN_USE:
            return self.Command('r',open)
        else:
            return self.Command('l',open)


class MoveBase:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("waiting for move_base action...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")

        self.PoseInitializer = octomap_helpers.PoseInitializer()
        self.OctomapClient = octomap_helpers.OctomapClient()

    def InitializePosePR2(self):
        self.PoseInitializer.setInitialPosePR2()

    def clearOctomapCouch(self):
        self.OctomapClient.clearOctomapCouch()

    def MoveToPose(self, frame, input_pose):

        # clear octomap in front of PR2 that is created during manipulation
        self.OctomapClient.clearOctomapWorkspacePR2()

        # clear octomap along the navigation path of PR2 that is possibly created when sudden drop of control happened during previous navigation
        self.OctomapClient.clearOctomapPath()

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose = input_pose

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(120.0)):
            rospy.logwarn("Arrived to the goal!")
            return True
        else:
            self.client.cancel_all_goals()
            rospy.logwarn("Aborted! 120 seconds has passed after setting a goal for navigation...")
            return False

    def MoveToInternDesk(self):
        pose = geometry_msgs.msg.Pose()

        # HACK intern desk (around the right marker)
        pose.position.x = -1.06836
        pose.position.y = -1.24624
        pose.position.z = -1e-05
        pose.orientation.x = -0.0017
        pose.orientation.y = -0.0002
        pose.orientation.z = -0.70305
        pose.orientation.w = 0.71114

        return self.MoveToPose("map", pose)

    def MoveToWorkstation(self):
        pose = geometry_msgs.msg.Pose()

        # HACK (lower) workstation

        # National Robotics Week
#         pose.position.x = 0.3489
#         pose.position.y = 0.5328
#         pose.position.z = 0.0
#         pose.orientation.x = 0.002
#         pose.orientation.y = 0.0003
#         pose.orientation.z = 0.69345
#         pose.orientation.w = 0.72049

        # RCTA Demo
#         pose.position.x = -0.5134
#         pose.position.x = -0.6134
        pose.position.x = -0.5634
        pose.position.y = 0.1364
        pose.position.z = 0.0
        pose.orientation.x = 0.002
        pose.orientation.y = 0.0003
        pose.orientation.z = -0.01074
        pose.orientation.w = 0.99994

        return self.MoveToPose("map", pose)


class MoveitMoveArm:
    def __init__(self, LARM_IN_USE):

        self.LARM_IN_USE = LARM_IN_USE    # True: manipulation with left arm, False: with right arm

        rospy.loginfo("bringing up move_arm...")
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_robot_commander = moveit_commander.RobotCommander()
        self.moveit_planning_scene = moveit_commander.PlanningSceneInterface()
        
        if self.LARM_IN_USE:
            self.moveit_planning_group = moveit_commander.MoveGroupCommander("left_arm")
        else:
            self.moveit_planning_group = moveit_commander.MoveGroupCommander("right_arm")


        ############### MOTION PLANNER OPTIONS ###############

        # NOTE to switch the planner to ompl, need to set the parameter 'use_sbpl_pipeline' to 'false' in launch/pr2/tatooine_moveit_setup.launch
        # NOTE also make sure to execute 'rosparam delete /move_group' to prevent any possible confusion

        #use_sbpl_pipeline = False
        use_sbpl_pipeline = True

        ######################################################


        if use_sbpl_pipeline:
            if self.LARM_IN_USE:
                self.moveit_planning_group.set_planner_id("left_arm[arastar_bfs_manip]")  # slow; robust but sometimes rotating the wrist; initial plan can be found within a second, so just reduce allowed_planning_time
            else:
                self.moveit_planning_group.set_planner_id("right_arm[arastar_bfs_manip]")  # slow; robust but sometimes rotating the wrist; initial plan can be found within a second, so just reduce allowed_planning_time
                # self.moveit_planning_group.set_planner_id("right_arm[larastar_bfs_manip]")  # not faster; similar motion
                # self.moveit_planning_group.set_planner_id("right_arm[mhastar_bfs_manip]")  # not faster; similar motion
                # self.moveit_planning_group.set_planner_id("right_arm[arastar_bfs_workspace]")   # faster; weird suboptimal path
                # self.moveit_planning_group.set_planner_id("right_arm[arastar_euclid_workspace]")  # slow; wrist orientation converges early
                # self.moveit_planning_group.set_planner_id("right_arm[larastar_euclid_workspace]")   # not faster, abrupt motions!

            self.moveit_planning_group.set_planning_time(rospy.get_param("move_group/allowed_planning_time"))
            self.moveit_planning_group.allow_replanning(True)

            workspace_frame = rospy.get_param("move_group/workspace_frame")
            min_x = rospy.get_param("move_group/workspace_min/x")
            min_y = rospy.get_param("move_group/workspace_min/y")
            min_z = rospy.get_param("move_group/workspace_min/z")
            max_x = rospy.get_param("move_group/workspace_max/x")
            max_y = rospy.get_param("move_group/workspace_max/y")
            max_z = rospy.get_param("move_group/workspace_max/z")
            workspace = [min_x, min_y, min_z, max_x, max_y, max_z]
            self.moveit_planning_group.set_workspace(workspace)

            self.moveit_planning_group.set_goal_position_tolerance(rospy.get_param("move_group/tolerance/position"))
            self.moveit_planning_group.set_goal_orientation_tolerance(rospy.get_param("move_group/tolerance/orientation"))
            self.moveit_planning_group.set_goal_joint_tolerance(rospy.get_param("move_group/tolerance/joint"))

        else:
            self.moveit_planning_group.set_planner_id("RRTkConfigDefault")

        self.tflistener = tf.TransformListener()
        self.inserted_desks = []
        self.inserted_tables = []
        self.inserted_objects = []
        rospy.loginfo("Connected.")
        rospy.sleep(0.5);

    def Cleanup(self):
        moveit_commander.roscpp_shutdown()

    def MoveToPose(self, pose, reference_frame):
        # need to manually convert to odom_combined frame
        if(not self.tflistener.frameExists(reference_frame) or 
           not self.tflistener.frameExists("odom_combined")):
            rospy.logwarn("Warning: could not look up provided reference frame")
            return False
        input_ps = PoseStamped()
        input_ps.pose = pose
        input_ps.header.frame_id = reference_frame
        if reference_frame != 'odom_combined':
            correct_ps = self.tflistener.transformPose("odom_combined", input_ps )
        else:
            correct_ps = input_ps

        self.moveit_planning_group.set_start_state_to_current_state()

        self.moveit_planning_group.set_pose_target(correct_ps.pose)
        #self.moveit_planning_group.set_pose_reference_frame(reference_frame) # does not work
        plan=self.moveit_planning_group.plan()
        if not plan.joint_trajectory.points:
            return False
        rospy.sleep(3)
        self.moveit_planning_group.go(wait=True)
        return True


#     def MoveToHandoff(self):
#         pose = geometry_msgs.msg.Pose()
#         pose.position.x = 0.62
#         pose.position.y = 0
#         pose.position.z = 0.93
#         quat = quaternion_from_euler(math.pi/2,0,0)
#         pose.orientation.x = quat[0]
#         pose.orientation.y = quat[1]
#         pose.orientation.z = quat[2]
#         pose.orientation.w = quat[3]
#         return self.MoveToPose(pose, "base_footprint")

#     def MoveToHome(self):
#         pose = geometry_msgs.msg.Pose()
#         pose.position.x = 0.09
#         pose.position.y = -0.75
#         pose.position.z = 0.89
#         quat = quaternion_from_euler(0,0,0)
#         pose.orientation.x = quat[0]
#         pose.orientation.y = quat[1]
#         pose.orientation.z = quat[2]
#         pose.orientation.w = quat[3]
#         return self.MoveToPose(pose, "base_footprint")

    def MoveRightToWide(self):
        pose = geometry_msgs.msg.Pose()

        # original
#         pose.position.x = 0
#         pose.position.y = -0.64
#         pose.position.z = 1.05
#         pose.orientation.x = 0.0
#         pose.orientation.y = 0.0
#         pose.orientation.z = 0.0
#         pose.orientation.w = 1.0

        # RCTA demo
        # HACK XXX won't collide with nearby desk, but the gripper might be visible to kinect
        pose.position.x = 0.30
        pose.position.y = -0.34
        pose.position.z = 1.05
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return self.MoveToPose(pose, "base_footprint")

    def MoveRightToCarry(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.32
        pose.position.y = -0.608
        pose.position.z = 1.244
        quat = quaternion_from_euler(-1.679, -1.479, 1.154)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return self.MoveToPose(pose, "base_footprint")

    def MoveLeftToWide(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0
        pose.position.y = 0.64
        pose.position.z = 1.05
        quat = quaternion_from_euler(math.pi,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return self.MoveToPose(pose, "base_footprint")

    def MoveLeftToCarry(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.32
        pose.position.y = 0.608
        pose.position.z = 1.244
        quat = quaternion_from_euler(-1.679, -1.479, 1.154)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return self.MoveToPose(pose, "base_footprint")

    def MoveArmInUseToWide(self):
        if self.LARM_IN_USE:
            return self.MoveLeftToWide()
        else:
            return self.MoveRightToWide()

    def MoveArmInUseToCarry(self):
        if self.LARM_IN_USE:
            return self.MoveLeftToCarry()
        else:
            return self.MoveRightToCarry()


# 1) place an object as the same as it was picked up
    def MoveRightToExtend(self, release_pose):
        return self.MoveToPose(release_pose, "base_footprint")

    def MoveRightToShortExtend(self, release_pose):
        return self.MoveToPose(release_pose, "base_footprint")

    # def MoveLeftToExtend(self, release_pose):
    #     return self.MoveToPose(release_pose, "base_footprint")

    # def MoveLeftToShortExtend(self, release_pose):
    #     return self.MoveToPose(release_pose, "base_footprint")

# 2) place an object at a pre-defined pose
    # def MoveRightToExtend(self, release_pose):
        # return self.MoveToPose(release_pose, "base_footprint")

    # def MoveRightToShortExtend(self, release_pose):
        # return self.MoveToPose(release_pose, "base_footprint")

    def MoveLeftToExtend(self, release_pose):
        # HACK hard-coded
        release_pose.position.x = 0.570212716415
        release_pose.position.y = 0.427063007244
        release_pose.position.z = 0.9882137459
        release_pose.orientation.x = 0.659906741331
        release_pose.orientation.y = -0.300090039643
        release_pose.orientation.z = -0.609382718567
        release_pose.orientation.w = -0.321125774682
        return self.MoveToPose(release_pose, "base_footprint")

    def MoveLeftToShortExtend(self, release_pose):
        # HACK hard-coded
        release_pose.position.x = 0.570212716415
        release_pose.position.y = 0.427063007244
        release_pose.position.z = 1.0882137459
        release_pose.orientation.x = 0.659906741331
        release_pose.orientation.y = -0.300090039643
        release_pose.orientation.z = -0.609382718567
        release_pose.orientation.w = -0.321125774682
        return self.MoveToPose(release_pose, "base_footprint")


    def MoveArmInUseToExtend(self, release_pose):
        release_pose_rev = copy.deepcopy(release_pose)

        # HACK assuming that we picked up the object from table and put it down on desk
        table_height = 0.32
        desk_height = 0.70
        release_pose_rev.position.z += desk_height - table_height

        # HACK to allow more margin with desk
        release_pose_rev.position.z += 0.005

        if self.LARM_IN_USE:
            return self.MoveLeftToExtend(release_pose_rev)
        else:
            return self.MoveRightToExtend(release_pose_rev)

    def MoveArmInUseToShortExtend(self, release_pose):
        release_pose_rev = copy.deepcopy(release_pose)

        # HACK assuming that we picked up the object from table and put it down on desk
        table_height = 0.32
        desk_height = 0.70
        release_pose_rev.position.z += desk_height - table_height

        # HACK to allow more margin with desk
        release_pose_rev.position.z += 0.005

        # HACK retract along the x-axis of the r/l_wrist_roll_link frame
        factor_wrist_x = -0.15
        if self.LARM_IN_USE:
            (wrist_trans, wrist_quat) = self.tflistener.lookupTransform("base_footprint", "l_wrist_roll_link", rospy.Time())
        else:
            (wrist_trans, wrist_quat) = self.tflistener.lookupTransform("base_footprint", "r_wrist_roll_link", rospy.Time())
        wrist_matrix = self.tflistener.fromTranslationRotation(wrist_trans, wrist_quat)
        wrist_offset_x = wrist_matrix[:3,0] * factor_wrist_x
        release_pose_rev.position.x += wrist_offset_x[0]
        release_pose_rev.position.y += wrist_offset_x[1]
        release_pose_rev.position.z += wrist_offset_x[2]

        if self.LARM_IN_USE:
            return self.MoveLeftToShortExtend(release_pose_rev)
        else:
            return self.MoveRightToShortExtend(release_pose_rev)

    def AddCollisionObject(self, name, posestamped, input_size):
        self.moveit_planning_scene.add_box(name, posestamped, size=input_size)
        self.inserted_objects.append(name)

    def AddDeskCollisionObject(self, name, pose_in_map):
        # HACK hard-coded

        # adjust offset of marker on the desk
        pose_in_map.pose.position.x += -0.44
        pose_in_map.pose.position.y += -0.30
        pose_in_map.pose.position.z = 0.35

        pose_in_map.pose.orientation.x = 0
        pose_in_map.pose.orientation.y = 0
        quat_z = pose_in_map.pose.orientation.z
        quat_w = pose_in_map.pose.orientation.w
        quat_norm = math.sqrt(quat_z**2 + quat_w**2)
        pose_in_map.pose.orientation.z = quat_z / quat_norm
        pose_in_map.pose.orientation.w = quat_w / quat_norm
        # WARN voxelized collision model for planner may be larger than this size!
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.67, 0.52, 0.7))

        rospy.loginfo("Added desk object %s", name)
        self.inserted_desks.append(name)

    def AddTableCollisionObject(self, name, pose_in_map):
        # HACK hard-coded

        # adjust offset of marker on the table
        pose_in_map.pose.position.z = 0.36

        pose_in_map.pose.orientation.x = 0
        pose_in_map.pose.orientation.y = 0
        quat_z = pose_in_map.pose.orientation.z
        quat_w = pose_in_map.pose.orientation.w
        quat_norm = math.sqrt(quat_z**2 + quat_w**2)
        pose_in_map.pose.orientation.z = quat_z / quat_norm
        pose_in_map.pose.orientation.w = quat_w / quat_norm
        # WARN voxelized collision model for planner may be larger than this size!
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.92, 0.77, 0.39))      # (lower) workstation

        rospy.loginfo("Added table object %s", name)
        self.inserted_tables.append(name)

    def AddDeskCollisionObjectInMap(self):  # intern desk (at original place)
        # HACK hard-coded
        pose_in_map = PoseStamped()
        pose_in_map.header.frame_id = "map"

        # National Robotics Week
#         pose_in_map.pose.position.x = -0.75
#         pose_in_map.pose.position.y = -2.20
#         pose_in_map.pose.position.z = 0.35
#         pose_in_map.pose.orientation.x = 0
#         pose_in_map.pose.orientation.y = 0
#         pose_in_map.pose.orientation.z = 0
#         pose_in_map.pose.orientation.w = 1

        # RCTA Demo
        pose_in_map.pose.position.x = -0.75
        pose_in_map.pose.position.y = -2.20
        pose_in_map.pose.position.z = 0.35
        pose_in_map.pose.orientation.x = 0
        pose_in_map.pose.orientation.y = 0
        pose_in_map.pose.orientation.z = 0
        pose_in_map.pose.orientation.w = 1

        name = "desk_0"
        # WARN voxelized collision model for planner may be larger than this size!
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(1.52, 0.67, 0.60))
        rospy.loginfo("Added desk object %s", name)
        self.inserted_desks.append(name)

    def AddDeskCollisionObjectInMapStationary(self):  # intern desk (next to the workstation)
        # HACK hard-coded
        pose_in_map = PoseStamped()
        pose_in_map.header.frame_id = "map"

        # RCTA Demo
        pose_in_map.pose.position.x = -0.65
        pose_in_map.pose.position.y = -0.45
        pose_in_map.pose.position.z = 0.715/2.0
        pose_in_map.pose.orientation.x = -0.0017
        pose_in_map.pose.orientation.y = -0.0002
        pose_in_map.pose.orientation.z = -0.70305
        pose_in_map.pose.orientation.w = 0.71114

        name = "desk_0"
        # WARN voxelized collision model for planner may be larger than this size!
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.91, 0.91, 0.70))
        rospy.loginfo("Added desk object %s", name)
        self.inserted_desks.append(name)

    def AddTableCollisionObjectInMap(self):  # workstation
        # HACK hard-coded
        pose_in_map = PoseStamped()
        pose_in_map.header.frame_id = "map"
        name = "table_0"

        # National Robotics Week
#         pose_in_map.pose.position.x = 0.0
#         pose_in_map.pose.position.y = 1.40
#         pose_in_map.pose.position.z = 0.195
#         pose_in_map.pose.orientation.x = 0
#         pose_in_map.pose.orientation.y = 0
#         pose_in_map.pose.orientation.z = 0
#         pose_in_map.pose.orientation.w = 1
#         # WARN voxelized collision model for planner may be larger than this size!
#         self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.92, 0.77, 0.30))

        # RCTA Demo
        pose_in_map.pose.position.x = 0.25
#         pose_in_map.pose.position.y = 0.39
        pose_in_map.pose.position.y = 0.45
        pose_in_map.pose.position.z = 0.195
#         pose_in_map.pose.orientation.x = 0
#         pose_in_map.pose.orientation.y = 0
#         pose_in_map.pose.orientation.z = 0
#         pose_in_map.pose.orientation.w = 1
        pose_in_map.pose.orientation.x = 0
        pose_in_map.pose.orientation.y = 0
        pose_in_map.pose.orientation.z = -0.070
        pose_in_map.pose.orientation.w = 0.998
        # WARN voxelized collision model for planner may be larger than this size!
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.77, 0.92, 0.30))

        rospy.loginfo("Added table object %s", name)
        self.inserted_tables.append(name)

    def AddRomanCollisionObjectInMap(self):
        # HACK hard-coded
        pose_in_map = PoseStamped()
        pose_in_map.header.frame_id = "map"

        # National Robotics Week
        pose_in_map.pose.position.x = -0.75+0.25
        pose_in_map.pose.position.y = 1.10
        pose_in_map.pose.position.z = 0.70
        pose_in_map.pose.orientation.x = 0
        pose_in_map.pose.orientation.y = 0
        pose_in_map.pose.orientation.z = 0
        pose_in_map.pose.orientation.w = 1

        name = "table_1"    # this is Roman
        # WARN voxelized collision model for planner may be larger than this size!
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.050, 0.50, 1.40))
        rospy.loginfo("Added Roman collision object %s", name)
        self.inserted_tables.append(name)

    def removeDeskObjects(self):
        for desk in self.inserted_desks:
            self.moveit_planning_scene.remove_world_object(desk)
        self.inserted_desks = []

    def removeTableObjects(self):
        for table in self.inserted_tables:
            self.moveit_planning_scene.remove_world_object(table)
        self.inserted_tables = []

    def removeAllObjects(self):
        for obj in self.inserted_objects:
            self.moveit_planning_scene.remove_world_object(obj)
        self.inserted_objects = []

    def removeObject(self, name):
        rospy.logwarn("method removeObject not implemented")

    def removeAllObjectsAndDesks(self):
        self.removeAllObjects()
        self.removeDeskObjects()
        self.removeTableObjects()


class RoconMoveArm:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('rocon_move_arm', RoconMoveArmAction)
        self.client.wait_for_server()
        rospy.loginfo("RoconMoveArm online")

    def MoveToHandoff(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.62
        pose.position.y = 0
        pose.position.z = 0.93
        quat = quaternion_from_euler(math.pi/2,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose(pose)

    def MoveToHome(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.09
        pose.position.y = -0.94
        pose.position.z = 0.89
        quat = quaternion_from_euler(0,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose(pose)

    def MoveToPose(self, pose):
        rospy.loginfo("RoconMoveArm is moving to Pose")
        goal = RoconMoveArmGoal()
        goal.target_pose = pose
        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(15.0)):
            result = self.client.get_result()
            return result.success
        return False


class ArmJointTrajAction:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

    def MoveArmToJoint(self, angles):
        goal = JointTrajectoryGoal()
        char = self.name[0]  # either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                                       char+'_shoulder_lift_joint',
                                       char+'_upper_arm_roll_joint',
                                       char+'_elbow_flex_joint',
                                       char+'_forearm_roll_joint',
                                       char+'_wrist_flex_joint',
                                       char+'_wrist_roll_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

class ArmJointCommand:
    def __init__(self, LARM_IN_USE):

        self.LARM_IN_USE = LARM_IN_USE    # True: manipulation with left arm, False: with right arm

        self.RightArmJointCommand = ArmJointTrajAction('r_arm')
        self.LeftArmJointCommand = ArmJointTrajAction('l_arm')

    # NOTE current joint values can be read from 'rostopic echo /[r/l]_arm_controller/state'

    def MoveRightArmToWide(self):
        self.RightArmJointCommand.MoveArmToJoint([-1.6583625690312713, 0.6874497663917372, -0.3591542852171954, -2.0396477595207805, 4.633907864244298, -1.2780054373789036, 2.9439450216806975])

    def MoveRightArmToMiddle(self):
        self.RightArmJointCommand.MoveArmToJoint([-1.5590404414376025, 1.0836077912898985, -1.0852409390200402, -2.0560068499071193, -2.230748097091767, -0.6395664210674809, -3.351435931429788])

    def MoveRightArmToSide(self):
        self.RightArmJointCommand.MoveArmToJoint([-2.134992712216583, 1.046132240354625, -2.2202324000660947, -1.9038528322430315, -2.7226797245193337, -0.10465688013304186, 4.67507793460336])

    def MoveLeftArmToWide(self):
        self.LeftArmJointCommand.MoveArmToJoint([1.6327356580271766, 0.16224827869037478, 0.13903629877154455, -1.794550945148468, 1.5893983104724974, -1.3209737500667504, -0.03455158539307579])

    def MoveLeftArmToWide(self):
        # self.LeftArmJointCommand.MoveArmToJoint([])
        rospy.logerr("MoveLeftArmToWide() is not yet supported!")

    def MoveLeftArmToSide(self):
        self.LeftArmJointCommand.MoveArmToJoint([2.050585009506385, 1.168843268710281, 2.0143859931673638, -1.691908510777547, 1.264066293462545, -0.09850362202844609, 0.00586820137944688])

    def MoveArmInUseToWide(self):   # high height, far from the body
        if self.LARM_IN_USE:
            self.MoveLeftArmToWide()
        else:
            self.MoveRightArmToWide()

    def MoveArmInUseToMiddle(self): # middle height, moderately close to the body
        if self.LARM_IN_USE:
            self.MoveLeftArmToMiddle()
        else:
            self.MoveRightArmToMiddle()

    def MoveArmInUseToSide(self):   # low height, close to the body
        if self.LARM_IN_USE:
            self.MoveLeftArmToSide()
        else:
            self.MoveRightArmToSide()

    def MoveArmNotInUseToWide(self):
        if self.LARM_IN_USE:
            self.MoveRightArmToWide()
        else:
            self.MoveLeftArmToWide()

    def MoveArmNotInUseToSide(self):
        if self.LARM_IN_USE:
            self.MoveRightArmToSide()
        else:
            self.MoveLeftArmToSide()

    def MoveRightArmToPreRelease(self):
        # self.RightArmJointCommand.MoveArmToJoint([])
        rospy.logerr("MoveRightArmToPreRelease() is not yet supported!")

    def MoveRightArmToRelease(self):
        # self.RightArmJointCommand.MoveArmToJoint([])
        rospy.logerr("MoveRightArmToRelease() is not yet supported!")

    def MoveLeftArmToPreRelease(self):
        # National Robotics Week
#         self.LeftArmJointCommand.MoveArmToJoint([0.3728037940530639, -0.03705736014372039, 1.45603048774742, -0.4882848163348973, 7.963510974467809, -1.5628833458204952, -6.2780816196848725])
        # RCTA Demo
        self.LeftArmJointCommand.MoveArmToJoint([0.37172600802742317, -0.07038775984687563, 1.3067401443970559, -1.2629528929481542, 1.5492524321288084, -1.7704100174655357, 0.03470862632335847])

    def MoveLeftArmToRelease(self):
        # National Robotics Week
#         self.LeftArmJointCommand.MoveArmToJoint([0.42014347256390283, 0.06462573742783409, 1.4170642649395377, -0.5162255636319181, 7.971956649162592, -1.5152845872315392, -6.2752100309583])
        # RCTA Demo
        self.LeftArmJointCommand.MoveArmToJoint([0.3807628293193347, 0.11825891867605831, 1.3785791066107649, -1.3063841063632124, 1.7470894694997257, -1.6744293397258303, 0.02487561038085051])

    def MoveArmInUseToPreRelease(self):
        if self.LARM_IN_USE:
            self.MoveLeftArmToPreRelease()
        else:
            self.MoveRightArmToPreRelease()

    def MoveArmInUseToRelease(self):
        if self.LARM_IN_USE:
            self.MoveLeftArmToRelease()
        else:
            self.MoveRightArmToRelease()

