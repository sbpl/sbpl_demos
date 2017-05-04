#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
import sbpl_demos.grasping_helpers as grasping_helpers
import sbpl_demos.perception_helpers as perception_helpers
from sbpl_demos.perception_helpers import AR_TYPES
from geometry_msgs.imsg import Pose
from sbpl_demos.msg import RoconMoveArmAction, RoconMoveArmGoal, RoconMoveArmResult
from sbpl_demos.msg import RoconPickAction, RoconPickGoal, RoconPickResult
from sbpl_demos.msg import RoconRobotiqAction, RoconRobotiqGoal, RoconRobotiqResult

from sbpl_demos.srv import StateMachine


if __name__ == "__main__":
    rospy.init_node('roman_grasp_test_v3')
    PickClient=actionlib.SimpleActionClient('/roman_pick_action', RoconPickAction)
    ArmClient=actionlib.SimpleActionClient('/roman_move_arm', RoconMoveArmAction)
    GraspClient=actionlib.SimpleActionClient('/roman_grasp_action', RoconRobotiqAction)
    rospy.loginfo("Waiting for servers...")
    PickClient.wait_for_server()
    ArmClient.wait_for_server()
    GraspClient.wait_for_server()
    rospy.loginfo("Connected.")

    print "waiting for state machine server... "
    rospy.wait_for_service('state_machine')
    print "connected"
    StateMachineClient = rospy.ServiceProxy('state_machine', StateMachine)
    StateMachineRequest = StateMachineRequest()

    '''
    demo loop
    '''
    
    print "waiting for object request"
    while not rospy.is_shutdown():
        StateMachineRequest.command= "Get"
        StateMachineRequest.request_key = "requested_object"
        StateMachineRequest.request_value = ""
        res = StateMachineClient(StateMachineRequest)

        if res.result_value != "" 
            objresult = res.result_value
            break

        else:
            rospy.sleep(1)

    print objresult " requested"






    '''
    demo done
    '''
    StateMachineRequest.command= "Set"
    StateMachineRequest.request_key = "ROMAN_STATE"
    StateMachineRequest.request_value = "DONE"
    res = StateMachineClient(StateMachineRequest)







    while not rospy.is_shutdown():
        # Start the demo
        test = False
        while(test):
#        while (service.get(object) != mustard):
#            wait some time
       
        else:     
            print "go home"
            ArmClient.send_goal(Home)
            ArmClient.wait_for_result()

            GraspClient.send_goal(Open)
            GraspClient.wait_for_result()

            print "pick"
            PickClient.send_goal(RoconPickGoal())
            result = PickClient.wait_for_result()
            if result:
                ArmClient.send_goal(Handoff)
                ArmClient.wait_for_result()

                GraspClient.send_goal(Open)
                GraspClient.wait_for_result()
