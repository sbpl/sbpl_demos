#!/usr/bin/env python
import sys
import roslib
import rospy
import tf
import math
import numpy

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_from_matrix


class PerchClient:
    def __init__(self):

        self.locked = True

        self.tflistener = tf.TransformListener()

        self.publisher = rospy.Publisher('/requested_object', String, queue_size=10)
        rospy.Subscriber("/perch_pose", Pose, self.perch_callback)


    def getGraspPose(self, requested_object):

        object_pose_in_base = self.getObjectPose(requested_object)

        # TODO provide multiple candidate poses


        # transformation matrix of object in /base_footprint frame
        object_pos_in_base = (object_pose_in_base.position.x, object_pose_in_base.position.y, object_pose_in_base.position.z)
        object_quat_in_base = (object_pose_in_base.orientation.x, object_pose_in_base.orientation.y, object_pose_in_base.orientation.z, object_pose_in_base.orientation.w)
        object_matrix_in_base = self.tflistener.fromTranslationRotation(object_pos_in_base, object_quat_in_base)

        # transformation matrix of object in /odom_combined frame
        (base_in_odom_pos, base_in_odom_quat) = self.tflistener.lookupTransform("odom_combined", "base_footprint", rospy.Time())
        base_matrix_in_odom = self.tflistener.fromTranslationRotation(base_in_odom_pos, base_in_odom_quat)
        object_matrix_in_odom = numpy.dot(base_matrix_in_odom, object_matrix_in_base)


        # hard-coded grasp pose in object local frame
        grasp_pos_in_local = (0.0, 0.23, 0.0)
        grasp_quat_in_local = quaternion_from_euler(0, 0, -math.pi/2.0)
        grasp_matrix_in_local = self.tflistener.fromTranslationRotation(grasp_pos_in_local, grasp_quat_in_local)

        # compute grasp pose in /odom_combined frame
        grasp_matrix_in_odom = numpy.dot(object_matrix_in_odom, grasp_matrix_in_local)

        # convert transformation matrix to geometry_msgs/Pose
        grasp_pose = Pose()
        grasp_pose.position.x = grasp_matrix_in_odom[0, 3]
        grasp_pose.position.y = grasp_matrix_in_odom[1, 3]
        grasp_pose.position.z = grasp_matrix_in_odom[2, 3]
        grasp_quat = quaternion_from_matrix(grasp_matrix_in_odom)
        grasp_pose.orientation.x = grasp_quat[0]
        grasp_pose.orientation.y = grasp_quat[1]
        grasp_pose.orientation.z = grasp_quat[2]
        grasp_pose.orientation.w = grasp_quat[3]


        # hard-coded grasp pose in object local frame
        interp_pos_in_local = (0.0, 0.33, 0.0)
        interp_quat_in_local = quaternion_from_euler(0, 0, -math.pi/2.0)
        interp_matrix_in_local = self.tflistener.fromTranslationRotation(interp_pos_in_local, interp_quat_in_local)

        # compute interp pose in global frame
        interp_matrix_in_odom = numpy.dot(object_matrix_in_odom, interp_matrix_in_local)

        # convert transformation matrix to geometry_msgs/Pose
        interp_pose = Pose()
        interp_pose.position.x = interp_matrix_in_odom[0, 3]
        interp_pose.position.y = interp_matrix_in_odom[1, 3]
        interp_pose.position.z = interp_matrix_in_odom[2, 3]
        interp_quat = quaternion_from_matrix(interp_matrix_in_odom)
        interp_pose.orientation.x = interp_quat[0]
        interp_pose.orientation.y = interp_quat[1]
        interp_pose.orientation.z = interp_quat[2]
        interp_pose.orientation.w = interp_quat[3]


        # return the results
        return (grasp_pose, interp_pose)


    def getObjectPose(self, requested_object):

        self.locked = True

#         self.requested_object = String("006_mustard_bottle")
#         self.requested_object = String("011_banana")
#         self.requested_object = String("019_pitcher_base")
#         self.requested_object = String("024_bowl")

        self.requested_object = String(requested_object)

        self.send_request()
        tic = rospy.Time.now()
        rospy.loginfo("Sent object recognition request...")

        while not rospy.is_shutdown():
            if not self.locked:
                rospy.loginfo("Received object recognition result...")
#                 return self.get_grasp_pose(self.requested_object, self.perch_pose)
                return self.perch_pose

            else:
                toc = rospy.Time.now()
                if tic.to_sec() - toc.to_sec() > 30:
                    rospy.logwarn("Could not object recognition result in 30 sec... Returning a dummy pose")
                    return Pose()

                rospy.sleep(1)


    def send_request(self):

            self.publisher.publish(self.requested_object)
            self.locked = True


    def perch_callback(self, data):

        if self.locked:

            self.perch_pose = data
            print self.perch_pose
            self.locked = False


    def get_grasp_pose(self, requested_object, perch_pose):

#         grasp_pose = Pose()
# #         if requested_object is "006_mustard_bottle":
# #         rotz = 20/180*math.pi
#         rotz = 0/180*math.pi
#         grasp_pose.position.x = perch_pose.position.x*math.cos(rotz) - perch_pose.position.y*math.sin(rotz)
#         grasp_pose.position.x = perch_pose.position.x*math.sin(rotz) + perch_pose.position.y*math.cos(rotz)
#         grasp_pose.position.z = perch_pose.position.z
#         grasp_pose.orientation.x = perch_pose.orientation.x
#         grasp_pose.orientation.y = perch_pose.orientation.y
#         grasp_pose.orientation.z = perch_pose.orientation.z
#         grasp_pose.orientation.w = perch_pose.orientation.w
#
#
# #         elif requested_object is "011_banana":
# #         elif requested_object is "019_pitcher_base":
# #         elif requested_object is "024_bowl":
#
#         return grasp_pose
        return perch_pose



# if __name__ == "__main__":
#
#     rospy.init_node('object_localizer_client')
#
#     perch_client = PerchClient()
#
#     while not rospy.is_shutdown():
#
#         rospy.sleep(1)
#         print "in the loop.."
#
#         if not perch_client.locked:
#             perch_client.send_request()

