#!/usr/bin/env python
import sys
import roslib
import rospy
import tf

from std_srvs.srv import EmptyRequest, Empty
from octomap_msgs.srv import BoundingBoxQueryRequest, BoundingBoxQuery
from geometry_msgs.msg import Point, PoseStamped

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
        # HACK hard-coded bounding box to be cleared in /base_footprint frame
        min_in_base = Point()
        max_in_base = Point()

        # looser bound
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
        # HACK hard-coded bounding box to be cleared in /base_footprint frame
        min_in_map = Point()
        min_in_map.x = -3.0
        min_in_map.y =  0.5
        min_in_map.z =  0.0

        max_in_map = Point()
        max_in_map.x = -1.0
        max_in_map.y =  3.0
        max_in_map.z =  2.0

        self.clearOctomapWorkspace(min_in_map, max_in_map)


    def clearOctomapPath(self):
        # HACK hard-coded bounding boxes to be cleared in /map frame

		# National Robotics Week
#         min_in_map0 = Point()
#         min_in_map0.x = -0.4
#         min_in_map0.y = -1.8
#         min_in_map0.z =  0.0
#         max_in_map0 = Point()
#         max_in_map0.x = 0.8
#         max_in_map0.y = 0.8
#         max_in_map0.z = 2.0
#         self.clearOctomapWorkspace(min_in_map0, max_in_map0)
#
#         min_in_map1 = Point()
#         min_in_map1.x = -1.6
#         min_in_map1.y = -1.8
#         min_in_map1.z =  0.0
#         max_in_map2 = Point()
#         max_in_map2.x =  0.0
#         max_in_map2.y = -0.4
#         max_in_map2.z =  2.0
#         self.clearOctomapWorkspace(min_in_map1, max_in_map2)

		# RCTA Demo
        min_in_map0 = Point()
        min_in_map0.x = -1.45
        min_in_map0.y = -1.8
        min_in_map0.z =  0.0
        max_in_map0 = Point()
        max_in_map0.x = -0.25
        max_in_map0.y = 1.5
        max_in_map0.z = 2.0
        self.clearOctomapWorkspace(min_in_map0, max_in_map0)

        min_in_map1 = Point()
        min_in_map1.x = -1.6
        min_in_map1.y = -1.8
        min_in_map1.z =  0.0
        max_in_map2 = Point()
        max_in_map2.x =  0.0
        max_in_map2.y = -0.4
        max_in_map2.z =  2.0
        self.clearOctomapWorkspace(min_in_map1, max_in_map2)


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


if __name__ == "__main__":

    rospy.init_node('octomap_client')
    octomap_client = OctomapClient()

#     rospy.loginfo("Clearing partial octomap in front of PR2!")
#     octomap_client.clearOctomapWorkspacePR2()

    rospy.loginfo("Clearing partial octomap along the pathway for PR2!")
    octomap_client.clearOctomapPath()

