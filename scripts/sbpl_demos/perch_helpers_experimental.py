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
from sbpl_demos import grasp_return 

class PerchClient:

    def __init__(self):

        self.locked = True
        self.requested = True    # just for information
        self.requested_object = ""

        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()

        self.publisher = rospy.Publisher('/requested_object', String, queue_size=10)
        rospy.sleep(1)

        #Clean up and Populate grasp database
        grasp_return.getGraspsFromDatabase()
        rospy.Subscriber("/perch_pose", Pose, self.perch_callback)
        rospy.Subscriber("/requested_object", String, self.web_callback)    # just for information
        rospy.sleep(1)


    def getGraspPoses(self, requested_object):

        object_pose_in_base = self.getObjectPose(requested_object)
        return self.compute_grasp_pose_in_odom(object_pose_in_base)


    def getGraspPosesSpin(self):

        object_pose_in_base = self.getObjectPoseSpin()
        return self.compute_grasp_pose_in_odom(object_pose_in_base)

    # def getGraspsFromDatabase():

    #     # get an instance of RosPack with the default search paths
    #     rospack = rospkg.RosPack()
    #     fid = file(rospack.get_path('sbpl_demos')+'/data/grasp_database/grasp_database.yaml')
        
    #     try:
    #         config = yaml.load(fid)
    #     except yaml.YAMLError, exc:
    #         print "Error in configuration file:", exc

    #     str_namespace = '/grasps'
    #     if rospy.has_param(str_namespace):
    #             rospy.delete_param(str_namespace)

    #     for key,value in config.items():
    #         print "\nItem: "+str(key)
    #         str_item = str_namespace+str('/')+str(key)
    #         print str_item
    #         print "Num_graps: "+str(len(config[key].items()))
    #         rospy.set_param(str_item+'/num_grasps', len(config[key].items()))

    #         for grasp_i_key, grasp_i_value in config[key].items():
    #             grasp_index = str(grasp_i_key)[-1]
    #             grasp_n = str_item+'/'+grasp_index
    #             print "Grasp No: "+str(grasp_i_key)[-1]
    #             print value[grasp_i_key]['pregrasp']
    #             print value[grasp_i_key]['grasp']

    #             rospy.set_param(grasp_n+'/pregrasp/rot_x_y_z_w', value[grasp_i_key]['pregrasp']['rotation'])
    #             rospy.set_param(grasp_n+'/pregrasp/trans_x_y_z', value[grasp_i_key]['pregrasp']['translation'])

    #             rospy.set_param(grasp_n+'/grasp/rot_x_y_z_w', value[grasp_i_key]['grasp']['rotation'])
    #             rospy.set_param(grasp_n+'/grasp/trans_x_y_z', value[grasp_i_key]['grasp']['translation'])


    def compute_grasp_pose_in_odom(self, object_pose_in_base):

        # TODO provide multiple candidate poses
        # self.GraspReturn = grasp_return.GetGraspsFromDatabase()

        # transformation matrix of object in /base_footprint frame
        object_pos_in_base = (object_pose_in_base.position.x, object_pose_in_base.position.y, object_pose_in_base.position.z)
        object_quat_in_base = (object_pose_in_base.orientation.x, object_pose_in_base.orientation.y, object_pose_in_base.orientation.z, object_pose_in_base.orientation.w)
        object_matrix_in_base = self.tflistener.fromTranslationRotation(object_pos_in_base, object_quat_in_base)

        # transformation matrix of object in /odom_combined frame
        (base_in_odom_pos, base_in_odom_quat) = self.tflistener.lookupTransform("odom_combined", "base_footprint", rospy.Time())
        base_matrix_in_odom = self.tflistener.fromTranslationRotation(base_in_odom_pos, base_in_odom_quat)
        object_matrix_in_odom = numpy.dot(base_matrix_in_odom, object_matrix_in_base)


        # result containers of geometry_msgs/Pose[]
        grasp_poses = []
        interp_poses = []
        distances_to_grasp = []


        # NOTE hard-coded grasp data; to be replaced by an interface function for getting grasp database
        interp_matrices_in_local = []
        grasp_matrices_in_local = []
        interp_pos_in_local = ()
        interp_quat_in_local = ()
        grasp_pos_in_local = ()
        grasp_quat_in_local = ()

        #String to hold parameter parent
        object_grasp = "/grasps/"+self.requested_object
        num_grasps = rospy.get_param(object_grasp+"/num_grasps")

        for idx_grasp in range(1, num_grasps+1):
            interp_pos_in_local = rospy.get_param(object_grasp+"/"+str(idx_grasp)+"/pregrasp/trans_x_y_z")
            interp_quat_in_local = rospy.get_param(object_grasp+"/"+str(idx_grasp)+"/pregrasp/rot_x_y_z_w")
            grasp_pos_in_local = rospy.get_param(object_grasp+"/"+str(idx_grasp)+"/grasp/trans_x_y_z")
            grasp_quat_in_local = rospy.get_param(object_grasp+"/"+str(idx_grasp)+"/grasp/rot_x_y_z_w")

            interp_matrix_in_local = self.tflistener.fromTranslationRotation(interp_pos_in_local, interp_quat_in_local)
            grasp_matrix_in_local = self.tflistener.fromTranslationRotation(grasp_pos_in_local, grasp_quat_in_local)

            # add to the database
            interp_matrices_in_local.append(interp_matrix_in_local)
            grasp_matrices_in_local.append(grasp_matrix_in_local)

        # if self.requested_object == "003_cracker_box":
        #     for idx_grasp in range(1):
        #         # hard-coded grasp data for multiple grasp poses
        #         if idx_grasp == 0:
        #             # interp_pos_in_local = (-0.07186082, -0.32603487,  0.06163173)
        #             # interp_quat_in_local = (-0.11012889,  0.03911699,  0.60143071,  0.79033068)
        #             # grasp_pos_in_local = (-0.04971018, -0.22475378,  0.02199482)
        #             # grasp_quat_in_local = (-0.05106686,  0.02021765,  0.62407797,  0.77942935)


        #             # interp_pos_in_local = (0.0, 0.33, 0.0)
        #             # interp_quat_in_local = quaternion_from_euler(0, 0, -math.pi/2.0)
        #             # grasp_pos_in_local = (0.0, 0.23, 0.0)
        #             # grasp_quat_in_local = quaternion_from_euler(0, 0, -math.pi/2.0)
        #             interp_pos_in_local = (-0.0337354,   0.32518866,  0.33009632)
        #             interp_quat_in_local = (0.37145278,  0.33149848, -0.56646793,  0.65669298)
        #             grasp_pos_in_local = (-0.02731046,  0.2078597,   0.05684458)
        #             grasp_quat_in_local = (0.03516512,  0.03270248, -0.68649274,  0.72554923)
        #         elif idx_grasp == 1:
        #             # interp_pos_in_local = (0.0, -0.33, 0.0)
        #             # interp_quat_in_local = quaternion_from_euler(0, 0, math.pi/2.0)
        #             # grasp_pos_in_local = (0.0, -0.23, 0.0)
        #             # grasp_quat_in_local = quaternion_from_euler(0, 0, math.pi/2.0)
        #             interp_pos_in_local = (-0.25214457, -0.39110131,  0.05872603)
        #             interp_quat_in_local = (0.45390299,  0.8782439, -0.09746262,  0.11472036)
        #             grasp_pos_in_local = (-0.30422981, -0.33671074,  0.04042232)
        #             grasp_quat_in_local = (0.50179907,  0.84884832, -0.09401019,  0.13717255)
        #         else:
        #             # interp_quat_in_local = (0.45390299,  0.8782439, -0.09746262,  0.11472036)
        #             # interp_pos_in_local = (-0.25214457, -0.39110131,  0.05872603)
        #             # grasp_quat_in_local = (0.50179907,  0.84884832, -0.09401019,  0.13717255)
        #             # grasp_pos_in_local = (-0.30422981, -0.33671074,  0.04042232)
        #             interp_pos_in_local = (-0.04484351,  0.01072501,  0.31117956)
        #             interp_quat_in_local = (-0.55224775,  0.47379804,  0.41835077,  0.54361794)
        #             grasp_pos_in_local = (-0.03179835,  0.00880004,  0.24242511)
        #             grasp_quat_in_local = (-0.53407185,  0.50486571,  0.43946319,  0.51647845)      

        #         # transformation matrix of grasp pose in object local frame
        #         interp_matrix_in_local = self.tflistener.fromTranslationRotation(interp_pos_in_local, interp_quat_in_local)
        #         grasp_matrix_in_local = self.tflistener.fromTranslationRotation(grasp_pos_in_local, grasp_quat_in_local)

        #         # add to the database
        #         interp_matrices_in_local.append(interp_matrix_in_local)
        #         grasp_matrices_in_local.append(grasp_matrix_in_local)

        # elif self.requested_object == "006_mustard_bottle":
        #     for idx_grasp in range(0,2):
        #         # hard-coded grasp data for multiple grasp poses
        #         if idx_grasp == 0:
        #             interp_pos_in_local = (-0.23743967, 0.10060774, -0.01100586)
        #             interp_quat_in_local = (0.00853756, 0.01887509, -0.20440426, 0.97866733)
        #             grasp_pos_in_local = (-0.17370368, 0.08040001, -0.01967883)
        #             grasp_quat_in_local = (-0.00593715, 0.00969645, -0.21896115, 0.97566733)
        #         elif idx_grasp == 1:
        #             interp_pos_in_local = (0.21092159, -0.07302159, 0.00767459)
        #             interp_quat_in_local = (-0.04094911, -0.00174438, 0.98468821, 0.16943806)
        #             grasp_pos_in_local = (0.16134278, -0.06030286, 0.00066045)
        #             grasp_quat_in_local = (-0.0273408, 0.00288966, 0.98149227, 0.18951796)

        #         # transformation matrix of grasp pose in object local frame
        #         interp_matrix_in_local = self.tflistener.fromTranslationRotation(interp_pos_in_local, interp_quat_in_local)
        #         grasp_matrix_in_local = self.tflistener.fromTranslationRotation(grasp_pos_in_local, grasp_quat_in_local)

        #         # add to the database
        #         interp_matrices_in_local.append(interp_matrix_in_local)
        #         grasp_matrices_in_local.append(grasp_matrix_in_local)
        # elif self.requested_object == "010_potted meat_can":
        #     for idx_grasp in range(0,2):
        #         if idx_grasp == 0:

        #         elif idx_grasp ==1:

        #         else:


        # check for data validity
        if len(grasp_matrices_in_local) != len(interp_matrices_in_local):
            rospy.logerr("The number of grasp and pre-grasp poses are different! PerchClient.getGraspPoses() will return invalid values...")
            return (grasp_poses, interp_poses)


        # get current end-effector position
        (ee_position, ee_quat) = self.tflistener.lookupTransform("odom_combined", "r_wrist_roll_link", rospy.Time())

        # compute grasp poses from the given data
        for idx_grasp in range(0, len(grasp_matrices_in_local)):

            # idx_grasp-th grasp pose
            interp_matrix_in_local = interp_matrices_in_local[idx_grasp]
            grasp_matrix_in_local = grasp_matrices_in_local[idx_grasp]

            # compute grasp pose in /odom_combined frame
            interp_matrix_in_odom = numpy.dot(object_matrix_in_odom, interp_matrix_in_local)
            grasp_matrix_in_odom = numpy.dot(object_matrix_in_odom, grasp_matrix_in_local)

            # convert transformation matrix to geometry_msgs/Pose
            interp_pose = self.convert_T_to_Pose(interp_matrix_in_odom)
            grasp_pose = self.convert_T_to_Pose(grasp_matrix_in_odom)

            # add to the result container
            interp_poses.append(interp_pose)
            grasp_poses.append(grasp_pose)

            # computed distance from the current end-effector pose to a grasp pose
            distance_to_grasp = self.compute_distance_to_grasp(ee_position, grasp_pose)
            distances_to_grasp.append(distance_to_grasp)

            # for visualization
            self.tfbroadcaster.sendTransform((grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z),
                (grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w),
                rospy.Time.now(), "grasp_pose_" + str(idx_grasp), "odom_combined")
            self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
                (interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
                rospy.Time.now(), "interp_pose_" + str(idx_grasp), "odom_combined")

        # return the results
        return (grasp_poses, interp_poses, distances_to_grasp)


    def getObjectPose(self, requested_object):

        self.locked = True

#         self.requested_object = "006_mustard_bottle"
#         self.requested_object = "011_banana"
#         self.requested_object = "019_pitcher_base"
#         self.requested_object = "024_bowl"

        self.requested_object = requested_object

        self.send_request()
        tic = rospy.Time.now()
        rospy.loginfo("Sent object recognition request...")

        while not rospy.is_shutdown():
            if not self.locked:
                rospy.loginfo("Received object recognition result...")
                return self.perch_pose

            else:
                toc = rospy.Time.now()
                if toc.to_sec() - tic.to_sec() > 120.0:
                    rospy.logwarn("Could not object recognition result in 120 sec... Returning a dummy pose")
                    return Pose()

                rospy.sleep(1)


    def getObjectPoseSpin(self):

        self.locked = True
        self.requested = False    # just for information

        tic = rospy.Time.now()

        while not rospy.is_shutdown():
            if not self.locked:
                rospy.loginfo("Received object recognition result from PERCH!")
                return self.perch_pose

            else:
                toc = rospy.Time.now()
                if not self.requested:
                    if (toc.to_sec() - tic.to_sec()) % 30.0 < 1.0:
                        rospy.loginfo("Waiting for user's object selection from tablet... %f sec has passed...", toc.to_sec() - tic.to_sec())

                rospy.sleep(1)


    def getRequestedObjectName(self):
        return self.requested_object


    def getRequestedObjectNameSpin(self):

        self.locked = False
        self.requested = False

        tic = rospy.Time.now()

        # self.requested_object = "003_cracker_box"
        # self.requested_object = "010_potted_meat_can"

        while not rospy.is_shutdown():
            if self.requested_object != "":
                rospy.loginfo("Received requested_object name from web: %s", self.requested_object)
                return self.requested_object

            else:
                toc = rospy.Time.now()
                if not self.requested:
                    if (toc.to_sec() - tic.to_sec()) % 30.0 < 1.0:
                        rospy.loginfo("Waiting for user's object selection from tablet... %f sec has passed...", toc.to_sec() - tic.to_sec())

                rospy.sleep(1)


    def send_request(self):

        self.publisher.publish(String(self.requested_object))
        rospy.sleep(3)
        self.locked = True


    def perch_callback(self, data):

        if self.locked:

            self.perch_pose = data
            # print self.perch_pose
            self.locked = False


    def web_callback(self, data):

        if not self.requested:
            self.requested_object = data.data
            rospy.loginfo("Received user's object selection: %s", self.requested_object)
            self.requested = True


    # convert transformation matrix to geometry_msgs/Pose
    def convert_T_to_Pose(self, T_matrix):
        pose = Pose()

        pose.position.x = T_matrix[0, 3]
        pose.position.y = T_matrix[1, 3]
        pose.position.z = T_matrix[2, 3]

        quat = quaternion_from_matrix(T_matrix)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose


    # computed distance from the current end-effector pose to a grasp pose
    def compute_distance_to_grasp(self, ee_position, grasp_pose):
        dx = grasp_pose.position.x - ee_position[0]
        dy = grasp_pose.position.y - ee_position[1]
        dz = grasp_pose.position.z - ee_position[2]
        distance_to_grasp = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance_to_grasp


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

