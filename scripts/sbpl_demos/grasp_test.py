#!/usr/bin/env python

import sys
import rospy
import tf
import numpy as np


if __name__ == "__main__":

	rospy.init_node("grasp_testing")

	rot = rospy.get_param("/grasps/006_mustard_bottle/2/pregrasp/rot_x_y_z_w")
	trans = rospy.get_param("/grasps/006_mustard_bottle/2/pregrasp/trans_x_y_z")
	rotMat = tf.transformations.quaternion_matrix(rot)

	transfMat = tf.TransformListener().fromTranslationRotation(trans,rot)

	print "PreGrasp Matrix:\n" + str(transfMat)

	print "Rotation Matrix:\n" + str(rotMat)
	print "Translation:\n" + str(trans)
	# rospy.set_param('/grasps/003_cracker_box/grasp1', config[0][0])


