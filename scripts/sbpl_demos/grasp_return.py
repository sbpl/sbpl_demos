import sys
import roslib
import rospy
import geometry_msgs.msg
import tf
import numpy as np
from sbpl_demos import perch_helpers
import yaml

roslib.load_manifest('sbpl_demos')



if __name__ == "main":

	fid = file('../data/grasp_database/grasp_database.yaml','r')

	yaml.load(fid)



