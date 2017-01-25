#! /usr/bin/env python
import sys
import rospy
import actionlib
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import PointStamped


rospy.init_node('move_head_action_client')
client=actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
client.wait_for_server()
		
def lookAt(frame_id, x, y, z):
	goal=PointHeadGoal()
	point=PointStamped()
	point.point.x=x
	point.point.y=y
	point.point.z=z
	point.header.frame_id=frame_id
	
	goal.target=point
	goal.pointing_frame="high_def_frame"
	goal.min_duration=rospy.Duration(0.5)
	goal.max_velocity=1.0

	client.send_goal(goal)
	client.wait_for_result()
	
	return

if __name__ == "__main__":
	if len(sys.argv) < 4:
		print "Usage: ./pr2_move_head_client.py [x coordinate] [y cooridnate] [z coordinate]"
		sys.exit()

	rospy.init_node('move_head_action_client')

	x=float(sys.argv[1])
	y=float(sys.argv[2])
	z=float(sys.argv[3])

	lookAt("base_link", x, y, z)
	print('sent goal!')	