#!/usr/bin/env python

import rospy

from sbpl_demos.srv import StateMachine

state = "WAIT"

def state_machine(request):
    if request.command == "Get":
        global state
        result = True
        result_state = state
        return result, result_state
    
    if request.command == "Set":
        state = request.request_state
        result_state = state
        if result_state == request.request_state:
            result = True
            return result, result_state
        else:
            result = False
            return result, result_state;

if __name__ == "__main__":
    rospy.init_node('state_machine');
    service = rospy.Service('state_machine', StateMachine, state_machine)

    # State 1: WAIT, State 2: ROMAN 3: PR2
    rospy.spin()
