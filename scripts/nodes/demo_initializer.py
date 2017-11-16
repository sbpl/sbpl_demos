#!/usr/bin/env python
import sys
import roslib
import rospy
import actionlib

# from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from control_msgs.msg import PointHeadGoal, PointHeadAction
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

# from sbpl_demos import octomap_helpers


# excerpted from scripts/sbpl_demos/pr2_helpers.py to avoid loading the whole PR2
def PointHead_LookAt(frame, x, y, z):
    client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
    rospy.loginfo("Waiting for point_head_action...")
    client.wait_for_server()
    rospy.loginfo("Connected.")

    goal = PointHeadGoal()
    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.frame_id = frame

    goal.target = point
    goal.pointing_frame = "high_def_frame"
    goal.min_duration = rospy.Duration(0.5)
    goal.max_velocity = 1.0

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(3.0)):
        return True
    else:
        return False


def PoseInitializer_setInitialPosePR2():
        # publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)    # NOTE for compatibility with Groovy, but not recommended
        rospy.sleep(0.5)

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"

        # pose in front of intern desk (around the right marker)
#         initial_pose.pose.pose.position.x = -1.06836
#         initial_pose.pose.pose.position.y = -1.14624
#         initial_pose.pose.pose.position.z = -1e-05
#         initial_pose.pose.pose.orientation.x = -0.0017
#         initial_pose.pose.pose.orientation.y = -0.0002
#         initial_pose.pose.pose.orientation.z = -0.70305
#         initial_pose.pose.pose.orientation.w = 0.71114

        # pose next to the workstation (for pr2_static demonstration)
        initial_pose.pose.pose.position.x = -0.4805
        initial_pose.pose.pose.position.y = 0.8371
        initial_pose.pose.pose.position.z = -1e-05
        initial_pose.pose.pose.orientation.x = -0.0017
        initial_pose.pose.pose.orientation.y = -0.0002
        initial_pose.pose.pose.orientation.z = -0.16441
        initial_pose.pose.pose.orientation.w = 0.98638

#         initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        initial_pose.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        publisher.publish(initial_pose)
        rospy.loginfo("Published /initialpose for PR2")


if __name__ == "__main__":

    rospy.init_node('demo_initializer')

    rospy.loginfo("Initialize PR2 heading")
    PointHead_LookAt('base_footprint', 1.0, 0.0, 0.0)

    # when amcl is running
    rospy.loginfo("Initialize PR2 pose in map")
    PoseInitializer_setInitialPosePR2()

############################################################

    # when amcl is running
#     rospy.loginfo("Initialize PR2 pose in map")
#     pose_initializer = octomap_helpers.PoseInitializer()
#     pose_initializer.setInitialPosePR2()

    # when move_base_node is running
#     rospy.loginfo("Clearing 2D costmap and 3D octomap")
#     costmap_client = octomap_helpers.CostmapClient()
#     costmap_client.clearCostmapAll()

    # when octomap_server is running
    # NOTE this will clear the whole costmap and a static octomap from file will not be up again
#     rospy.loginfo("Clearing 3D octomap")
#     octomap_client = octomap_helpers.OctomapClient()
#     octomap_client.clearOctomapAll()

