#!/usr/bin/env python

import sys
import roslib
import rospy
import rospkg
import geometry_msgs.msg
import tf
import numpy as np
from sbpl_demos import perch_helpers
import yaml

roslib.load_manifest('sbpl_demos')



if __name__ == "__main__":

	rospy.init_node('objects_to_grasps')
	# get an instance of RosPack with the default search paths
	rospack = rospkg.RosPack()
	fid = file(rospack.get_path('sbpl_demos')+'/data/grasp_database/grasp_database.yaml')
	print "hello"
	try:
		config = yaml.load(fid)
	except yaml.YAMLError, exc:
		print "Error in configuration file:", exc

	str_namespace = '/grasps'
	for key,value in config.items():
		print "\nItem: "+str(key)
		str_item = str_namespace+str('/')+str(key)
		print str_item
		print "Num_graps: "+str(len(config[key].items()))
		rospy.set_param(str_item+'/num_grasps', len(config[key].items()))

		for grasp_i_key, grasp_i_value in config[key].items():
			grasp_index = str(grasp_i_key)[-1]
			grasp_n = str_item+'/'+grasp_index
			print "Grasp No: "+str(grasp_i_key)[-1]
			print value[grasp_i_key]['pregrasp']
			print value[grasp_i_key]['grasp']

			rospy.set_param(grasp_n+'/pregrasp/rot_w_x_y_z', value[grasp_i_key]['pregrasp']['rotation'])
			rospy.set_param(grasp_n+'/pregrasp/trans_x_y_z', value[grasp_i_key]['pregrasp']['translation'])

			rospy.set_param(grasp_n+'/grasp/rot_w_x_y_z', value[grasp_i_key]['grasp']['rotation'])
			rospy.set_param(grasp_n+'/grasp/trans_x_y_z', value[grasp_i_key]['grasp']['translation'])


