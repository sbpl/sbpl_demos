#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from geometry_msgs.msg import Pose, PoseStamped
from sbpl_demos import perception_helpers
import sbpl_demos.pr2_helpers as pr2


if __name__ == "__main__":

	rospy.init_node('pr2_grasp_test')
	listener = tf.TransformListener()
	pr2_GripperCommand = pr2.GripperCommand()
	pr2_MoveitMoveArm = pr2.MoveitMoveArm()
	pr2_TorsoCommand = pr2.TorsoCommand()
	pr2_TuckArms = pr2.TuckArms()
	pr2_MoveBase = pr2.MoveBase()
	pr2_PointHead = pr2.PointHead()
	AR_listener = perception_helpers.ARTagListener()
	UpsampleGraspPoses = pr2.UpsampleGraspPoses()
	rospy.loginfo('All Action clients connected!')
	rospy.loginfo('Commanding Untucking')
	print "reference frame: ", pr2_MoveitMoveArm.moveit_planning_group.get_planning_frame()
	print "end effector: ", pr2_MoveitMoveArm.moveit_planning_group.get_end_effector_link()
	print "joints: ", pr2_MoveitMoveArm.moveit_planning_group.get_joints()
	print "current joint values: ", pr2_MoveitMoveArm.moveit_planning_group.get_current_joint_values()
	pr2_TuckArms.UntuckArms()
	#pr2_MoveitMoveArm.MoveToWide()
	#sys.exit(0)

	pr2_PointHead.LookAt("base_footprint", 1.25, 0, 0)
	rospy.loginfo('Commanding torso')
	pr2_TorsoCommand.MoveTorso(0.6)
	pr2_GripperCommand.Command('l', 0) #Close gripper
	

	rospy.sleep(1) #need a delay to set objects for some reason...
	rospy.loginfo('Commanding Tuckarms')
	pr2_TuckArms.TuckArms()
	rospy.loginfo('Commanding base to intern desk...')
	pr2_MoveBase.MoveToInternDesk()
	rospy.loginfo('Commanding Untucking')
	pr2_TuckArms.UntuckArms()
	rospy.loginfo('Commanding right gripper open')
	pr2_GripperCommand.Command('r', 1) #open grigger

	# add collision objects
	rospy.loginfo('Updating Collision Objects')
	pr2_MoveitMoveArm.AddDeskCollisionObjects()
	#print pr2_MoveitMoveArm.moveit_planning_scene.get_objects()

	## AR TAG PICKING HERE
	valid_poses = UpsampleGraspPoses.getValidPosesForCylinder(AR_listener.latest_pose.pose)
	if(len(valid_poses) > 0):
		pr2_MoveitMoveArm.AddCylinder(valid_poses[0])

		pr2_MoveitMoveArm.MoveToPose(valid_poses[0], "map")
		pr2_GripperCommand.Command('r', 0) #Close gripper

	rospy.loginfo('Tucking arms')
	pr2_TuckArms.TuckLeftArm()
	pr2_MoveitMoveArm.MoveToHome()
	rospy.loginfo('Commanding base to Workstation')
	pr2_MoveBase.MoveToWorkstation()
	rospy.loginfo('Untucking arms')
	pr2_TuckArms.UntuckArms()
	
	# add collision objects
	rospy.loginfo('Updating Collision Objects')
	pr2_MoveitMoveArm.AddDeskCollisionObjects()

	rospy.loginfo("Commanding right gripper Closed")
	pr2_GripperCommand.Command('r', 1) #open gripper
	rospy.loginfo('Commanding Tuckarms')
	
	pr2_GripperCommand.Command('r', 1) #Close Gripper
	pr2_TuckArms.TuckArms()

	#pr2_MoveitMoveArm.moveit_planning_scene.remove_world_object("intern_sphere")
	#pr2_MoveitMoveArm.moveit_planning_scene.remove_world_object("intern_desk")

	pr2_MoveitMoveArm.Cleanup()
	print('shutting down...')	