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
    def __init__(self):
        self.left_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', GripperCommandAction)
        self.right_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', GripperCommandAction)
        rospy.loginfo("waiting for gripper actions...")
        self.left_client.wait_for_server()
        self.right_client.wait_for_server()
        rospy.loginfo("Connected.")

    def Command(self, gripper, open):

        goal = GripperCommandGoal()

#         if open == 1:
#             goal.command.position = 0.09
#         else:
#             goal.command.position = 0.0
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

class MoveBase:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("waiting for move_base action...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")

        self.OctomapClient = octomap_helpers.OctomapClient()

    def MoveToPose(self, frame, input_pose):
        self.OctomapClient.clearOctomapWorkspacePR2()

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose = input_pose

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(120.0)):
            return True
        else:
            self.client.cancel_all_goals()
            return False

    def MoveToInternDesk(self):
        pose = geometry_msgs.msg.Pose()
#         pose.position.x = -0.29
#         pose.position.y = -2.5
#         pose.position.z = 0.0
#         quat = quaternion_from_euler(0,0,-math.pi/2.0)
#         pose.orientation.x = quat[0]
#         pose.orientation.y = quat[1]
#         pose.orientation.z = quat[2]
#         pose.orientation.w = quat[3]

#         pose.position.x = -0.478
#         pose.position.y = -1.122
#         pose.position.z = 0.0
#         pose.orientation.x = 0.0018
#         pose.orientation.y = 0.001
#         pose.orientation.z = -0.7134
#         pose.orientation.w = 0.70075

        pose.position.x = -1.1574
        pose.position.y = -1.1089
        pose.position.z = 0.0
        pose.orientation.x = 0.0008
        pose.orientation.y = 0.001
        pose.orientation.z = -0.6965
        pose.orientation.w = 0.7175

        self.MoveToPose("map", pose)

    def MoveToWorkstation(self):
        pose = geometry_msgs.msg.Pose()

#         # pose.position.x = 0.440
#         # pose.position.y = -1.540
#         # pose.position.z = 0.0
#         # quat = quaternion_from_euler(0,0,-0.004)
#         pose.position.x=-0.234
#         pose.position.y=-1.541
#         pose.position.z=0.0
#         quat = quaternion_from_euler(0,0,0.027)
#
#         pose.orientation.x = quat[0]
#         pose.orientation.y = quat[1]
#         pose.orientation.z = quat[2]
#         pose.orientation.w = quat[3]

#         pose.position.x = -0.6574
#         pose.position.y = -1.1089
#         pose.position.z = 0.0
#         pose.orientation.x = 0.0008
#         pose.orientation.y = 0.001
#         pose.orientation.z = -0.6965
#         pose.orientation.w = 0.7175

#         pose.position.x = 1.254
        pose.position.x = 0.854
        pose.position.y = 0.116
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        self.MoveToPose("map", pose)

    def MoveToWayPoint(self):
        # pose = geometry_msgs.msg.Pose()
        # pose.position.x = 0.005
        # pose.position.y = -0.257
        # pose.position.z = 0
        # quat = quaternion_from_euler(0,0,-1.104)
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.106
        pose.position.y = -1.161
        pose.position.z = 0
        quat = quaternion_from_euler(0, 0, -0.025)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose("map", pose)

class MoveitMoveArm:
    def __init__(self):
        rospy.loginfo("bringing up move_arm...")
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_robot_commander = moveit_commander.RobotCommander()
        self.moveit_planning_scene = moveit_commander.PlanningSceneInterface()
        
        self.moveit_planning_group = moveit_commander.MoveGroupCommander("right_arm")

        # NOTE to switch the planner to ompl, need to set the parameter 'use_sbpl_pipeline' to 'false' in launch/pr2/tatooine_moveit_setup.launch
        # NOTE also make sure to execute 'rosparam delete /move_group' to prevent any possible confusion
#         use_sbpl_pipeline = False
        use_sbpl_pipeline = True

        if use_sbpl_pipeline:
            self.moveit_planning_group.set_planner_id("right_arm[arastar_bfs_manip]")

            self.moveit_planning_group.set_planning_time(rospy.get_param("move_group/allowed_planning_time"))
            self.moveit_planning_group.allow_replanning(True)

            # TODO need to convert the reference frame of this workspace from /base_footprint to /odom_combined (or don't need to do it due to moveit's but?)
            workspace_frame = rospy.get_param("move_group/workspace_frame")
            min_x = rospy.get_param("move_group/workspace_min/x")
            min_y = rospy.get_param("move_group/workspace_min/y")
            min_z = rospy.get_param("move_group/workspace_min/z")
            max_x = rospy.get_param("move_group/workspace_max/x")
            max_y = rospy.get_param("move_group/workspace_max/y")
            max_z = rospy.get_param("move_group/workspace_max/z")
            workspace = [min_x, min_y, min_z, max_x, max_y, max_z]
            self.moveit_planning_group.set_workspace(workspace)    # CHECK header.frame_id is set to /odom_combined?

            self.moveit_planning_group.set_goal_position_tolerance(rospy.get_param("move_group/tolerance/position"))
            self.moveit_planning_group.set_goal_orientation_tolerance(rospy.get_param("move_group/tolerance/orientation"))
            self.moveit_planning_group.set_goal_joint_tolerance(rospy.get_param("move_group/tolerance/joint"))

        else:
            self.moveit_planning_group.set_planner_id("RRTkConfigDefault")

        self.tflistener = tf.TransformListener()
        self.inserted_desks = []
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
        # FIXME intensionally set the reference frame to /base_footprint to compensate the effect of bug in moveit?
        correct_ps = self.tflistener.transformPose("odom_combined", input_ps )

        self.moveit_planning_group.set_start_state_to_current_state()

        self.moveit_planning_group.set_pose_target(correct_ps.pose)
        #self.moveit_planning_group.set_pose_reference_frame(reference_frame) # DOES NOT WORK
        plan=self.moveit_planning_group.plan()
        if not plan.joint_trajectory.points:
            return False
        self.moveit_planning_group.go(wait=True)
        return True


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
        return self.MoveToPose(pose, "base_footprint")

    def MoveToHome(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.09
        pose.position.y = -0.75
        pose.position.z = 0.89
        quat = quaternion_from_euler(0,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return self.MoveToPose(pose, "base_footprint")

    def MoveRightToWide(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0
        pose.position.y = -0.64
        pose.position.z = 1.05
#         quat = quaternion_from_euler(-3.002, 0.117, 0.130)
#         pose.orientation.x = quat[0]
#         pose.orientation.y = quat[1]
#         pose.orientation.z = quat[2]
#         pose.orientation.w = quat[3]
#         pose.orientation.x = 0.00129958
#         pose.orientation.y = 0.0836125
#         pose.orientation.z = 0.0657168
#         pose.orientation.w = 0.994329
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

    def MoveRightToExtend(self, release_pose):
        pose = copy.deepcopy(release_pose)
        #pose.position.x = 0.58
        #pose.position.y = -0.11
        pose.position.z += 0.005
        return self.MoveToPose(pose, "base_footprint")

    def MoveRightToShortExtend(self, release_pose):
        pose = copy.deepcopy(release_pose)
        #pose.position.x = 0.58
        #pose.position.y = -0.11
        pose.position.z += 0.005

        # retract along the x-axis of the r_wrist_roll_link frame
        factor_wrist_x = -0.15
        (wrist_trans, wrist_quat) = self.tflistener.lookupTransform("base_footprint", "r_wrist_roll_link", rospy.Time())
        wrist_matrix = self.tflistener.fromTranslationRotation(wrist_trans, wrist_quat)
        wrist_offset_x = wrist_matrix[:3,0] * factor_wrist_x
        pose.position.x += wrist_offset_x[0]
        pose.position.y += wrist_offset_x[1]
        pose.position.z += wrist_offset_x[2]
        return self.MoveToPose(pose, "base_footprint")

    def AddCollisionObject(self, name, posestamped, input_size):
        self.moveit_planning_scene.add_box(name, posestamped, size=input_size)
        self.inserted_objects.append(name)

#     def AddDeskCollisionObject(self, name, posestamped):
#         posestamped.pose.position.y -= 0.35
#         posestamped.pose.position.z = 0.35
#         self.moveit_planning_scene.add_box(name, posestamped, size=(.67,1.52, .7) )
#         rospy.loginfo("Added desk object %s", name)
#         self.inserted_desks.append(name)

    def AddDeskCollisionObject(self, name, pose_in_map):

        # adjust offset of marker on the desk
        pose_in_map.pose.position.x += -0.44
        pose_in_map.pose.position.y += -0.30
        pose_in_map.pose.position.z = 0.35

        # constrain desk orientation
#         pose_in_map.pose.orientation.x = 0
#         pose_in_map.pose.orientation.y = 0
#         pose_in_map.pose.orientation.z = 0
#         pose_in_map.pose.orientation.w = 1
#         self.moveit_planning_scene.add_box(name, pose_in_map, size=(1.52, 0.67, 0.7))

        pose_in_map.pose.orientation.x = 0
        pose_in_map.pose.orientation.y = 0
        quat_z = pose_in_map.pose.orientation.z
        quat_w = pose_in_map.pose.orientation.w
        quat_norm = math.sqrt(quat_z**2 + quat_w**2)
        pose_in_map.pose.orientation.z = quat_z / quat_norm
        pose_in_map.pose.orientation.w = quat_w / quat_norm
        self.moveit_planning_scene.add_box(name, pose_in_map, size=(0.67, 1.52, 0.7))

        rospy.loginfo("Added desk object %s", name)
        self.inserted_desks.append(name)

    def removeDeskObjects(self):
        for desk in self.inserted_desks:
            self.moveit_planning_scene.remove_world_object(desk)
        self.inserted_desks = []

    def removeAllObjects(self):
        for obj in self.inserted_objects:
            self.moveit_planning_scene.remove_world_object(obj)
        self.inserted_objects = []

    def removeObject(self, name):
        rospy.logwarn("method removeObject not implemented")

    def removeAllObjectsAndDesks(self):
        self.removeAllObjects()
        self.removeDeskObjects()




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
    def __init__(self):
        self.RightArmJointCommand = ArmJointTrajAction('r_arm')
        self.LeftArmJointCommand = ArmJointTrajAction('l_arm')

    # NOTE current joint values can be read from 'rostopic echo /[r/l]_arm_controller/state'

    def MoveRightArmToWide(self):
        self.RightArmJointCommand.MoveArmToJoint([-1.6583625690312713, 0.6874497663917372, -0.3591542852171954, -2.0396477595207805, 4.633907864244298, -1.2780054373789036, 2.9439450216806975])

    def MoveLeftArmToWide(self):
        self.LeftArmJointCommand.MoveArmToJoint([2.050585009506385, 1.168843268710281, 2.0143859931673638, -1.691908510777547, 1.264066293462545, -0.09850362202844609, 0.00586820137944688])

