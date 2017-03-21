#!/usr/bin/env python
import sys
import roslib
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Pose


class PerchClient:
    def __init__(self):

        self.locked = True

        self.publisher = rospy.Publisher('/requested_object', String, queue_size=10)
        rospy.Subscriber("/perch_pose", Pose, self.perch_callback)


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

            # TODO make use of the object localization results for grasping

            self.locked = False


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

