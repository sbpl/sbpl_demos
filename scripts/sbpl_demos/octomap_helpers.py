#!/usr/bin/env python
import sys
import roslib
import rospy
import tf

from std_srvs.srv import EmptyRequest, Empty
from octomap_msgs.srv import BoundingBoxQueryRequest, BoundingBoxQuery
from geometry_msgs.msg import Point, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

class OctomapClient:

    def __init__(self):
        self.tflistener = tf.TransformListener()
        rospy.sleep(0.5)

        print("Waiting for octomap_server_combined/reset...")
        rospy.wait_for_service('/octomap_server_combined/reset')
        print("Connected.")
        self.client_all = rospy.ServiceProxy('/octomap_server_combined/reset', Empty)

        print("Waiting for octomap_server_combined/clear_bbx...")
        rospy.wait_for_service('/octomap_server_combined/clear_bbx')
        print("Connected.")
        self.client_bbx = rospy.ServiceProxy('/octomap_server_combined/clear_bbx', BoundingBoxQuery)


    def clearOctomapAll(self):
        request = EmptyRequest()
        if self.client_all(request):
            rospy.loginfo("Octomap was successfully cleared!")
        else:
            rospy.logwarn("Failed to clear octomap!")


    def clearOctomapWorkspacePR2(self):
        # hard-coded bounding box to be cleared in /base_footprint frame
        min_in_base = Point()
        max_in_base = Point()

#         min_in_base.x = -0.5
#         min_in_base.y = -1.0
#         min_in_base.z =  0.0
#         max_in_base.x =  1.0
#         max_in_base.y =  1.0
#         max_in_base.z =  2.0

        # tighter bound
        min_in_base.x = -0.5
        min_in_base.y = -0.9
        min_in_base.z =  0.0
        max_in_base.x =  0.668/2.0 + 0.10
        max_in_base.y =  0.9
        max_in_base.z =  2.0

        rospy.sleep(1)  # to void tf.ExtrapolationException

        try:
            # compute bounding box in /map frame
            min_in_base_stamped = PoseStamped()
            min_in_base_stamped.header.frame_id = "base_footprint"
            min_in_base_stamped.pose.position = min_in_base
            min_in_map_stamped = self.tflistener.transformPose("map", min_in_base_stamped)
            min_in_map = min_in_map_stamped.pose.position

            max_in_base_stamped = PoseStamped()
            max_in_base_stamped.header.frame_id = "base_footprint"
            max_in_base_stamped.pose.position = max_in_base
            max_in_map_stamped = self.tflistener.transformPose("map", max_in_base_stamped)
            max_in_map = max_in_map_stamped.pose.position

            self.clearOctomapWorkspace(min_in_map, max_in_map)

        except (tf.LookupException):
            print "tf.LookupException: Cannot find transform between /map and /base_footprint. Octomap will not be cleared!"
        except (tf.ConnectivityException):
            print "tf.ConnectivityException: Octomap will not be cleared!"
        except (tf.ExtrapolationException):
            print "tf.ExtrapolationException: Octomap will not be cleared!"


    def clearOctomapCouch(self):
        # hard-coded bounding box to be cleared in /base_footprint frame
        min_in_map = Point()
        min_in_map.x = -3.0
        min_in_map.y =  0.5
        min_in_map.z =  0.0

        max_in_map = Point()
        max_in_map.x = -1.0
        max_in_map.y =  3.0
        max_in_map.z =  2.0

        self.clearOctomapWorkspace(min_in_map, max_in_map)

    def clearOctomapWorkspace(self, min_in_map, max_in_map):
        request = BoundingBoxQueryRequest()
        request.min.x = min(min_in_map.x, max_in_map.x)
        request.min.y = min(min_in_map.y, max_in_map.y)
        request.min.z = min(min_in_map.z, max_in_map.z)
        request.max.x = max(min_in_map.x, max_in_map.x)
        request.max.y = max(min_in_map.y, max_in_map.y)
        request.max.z = max(min_in_map.z, max_in_map.z)
        if self.client_bbx(request):
            rospy.loginfo("Octomap was successfully cleared!")
        else:
            rospy.logwarn("Failed to clear octomap!")


class CostmapClient:

    def __init__(self):
        self.tflistener = tf.TransformListener()
        rospy.sleep(0.5)

        print("Waiting for move_base_node/clear_costmaps...")
        rospy.wait_for_service('/move_base_node/clear_costmaps')
        print("Connected.")
        self.client_all = rospy.ServiceProxy('/move_base_node/clear_costmaps', Empty)

    def clearCostmapAll(self):
        request = EmptyRequest()
        if self.client_all(request):
            rospy.loginfo("Costmap was successfully cleared!")
        else:
            rospy.logwarn("Failed to clear costmap!")


class PoseInitializer:

    def __init__(self):
        self.publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        # self.publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)    # NOTE for compatibility with Groovy, but not recommended
        rospy.sleep(0.5)

    def setInitialPosePR2(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"
        # pose in front of intern desk (around the right marker)
        initial_pose.pose.pose.position.x = -1.06836
        initial_pose.pose.pose.position.y = -1.14624
        initial_pose.pose.pose.position.z = -1e-05
        initial_pose.pose.pose.orientation.x = -0.0017
        initial_pose.pose.pose.orientation.y = -0.0002
        initial_pose.pose.pose.orientation.z = -0.70305
        initial_pose.pose.pose.orientation.w = 0.71114
#         initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        initial_pose.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(initial_pose)
        print("published /initialpose for PR2")


# if __name__ == "__main__":
#
#     rospy.init_node('octomap_client')
#     octomap_client = OctomapClient()
#
#     rospy.loginfo("Clearing partial octomap in front of PR2!")
#     octomap_client.clearOctomapWorkspacePR2()
#
#     rospy.sleep(2)
#
#     rospy.loginfo("Clearing all octomap!")
#     octomap_client.clearOctomapAll()

# if __name__ == "__main__":
#
#     rospy.init_node('pose_initializer')
#     pose_initializer = PoseInitializer()
#
#     rospy.loginfo("Setting initial pose of PR2 in front of intern desk!")
#     pose_initializer.setInitialPosePR2()

