#! /usr/bin/env python
import sys
import roslib
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from enum import Enum
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from sbpl_demos.srv import PoseUpsampleRequest, PoseUpsample
from sbpl_demos.msg import XYZRPY

class AR_TYPES(Enum):
	DESK = 1
	CYLINDER = 2
	CUBE = 3
	ROD_END = 4
	CUBOID_FLAT = 5
	CUBOID_EDGE = 6


class ARTagListener:
	def __init__(self):
		self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_get_pose)
		self.latest_markers=AlvarMarkers()
		self.last_reading = rospy.Time()
		'''
		self.desk_markers = AlvarMarkers()
		self.cylinder_markers = AlvarMarkers()
		self.cube_markers = AlvarMarkers()
		self.rod_end_markers = AlvarMarkers()
		self.cuboid_flat_markers = AlvarMarkers()
		self.cuboid_edge_markers = AlvarMarkers()
		'''

	def callback_get_pose(self,data):
		if len(data.markers) > 0:
			self.latest_markers = data
			self.last_reading = rospy.Time.now()

	def getMarkersAndCounts(self):
		# TODO check if the latest_reading is within some time tolerance, otherwise wildly
		# old markers could be used here
		n_desks = 0
		n_cylinders = 0
		n_cubes = 0
		n_rod_ends = 0
		n_cuboid_flats = 0
		n_cuboid_edges = 0

		if rospy.Time.now().to_sec() - self.last_reading.to_sec() < 2:		# self.latest_markers expires after 2 sec
			for marker in self.latest_markers.markers:
				print marker.id
				if marker.id == AR_TYPES.DESK.value:
					n_desks += 1
				elif marker.id == AR_TYPES.CYLINDER.value:
					n_cylinders += 1
				elif marker.id == AR_TYPES.CUBE.value:
					n_cubes += 1
				elif marker.id == AR_TYPES.ROD_END.value:
					n_rod_ends += 1
				elif marker.id == AR_TYPES.CUBOID_FLAT.value:
					n_cuboid_flats += 1
				elif marker.id == AR_TYPES.CUBOID_EDGE.value:
					n_cuboid_edges += 1

		return self.latest_markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges

	def getMarkersByType(self, markers, artype):
		type_markers = AlvarMarkers()
		for marker in markers.markers:
			if marker.id == artype.value:
				type_markers.header = markers.header
				type_markers.markers.append(marker)
		if len(type_markers.markers)> 0:
			return type_markers
		else:
			return False
