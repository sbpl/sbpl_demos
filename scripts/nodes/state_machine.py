#!/usr/bin/env python

import rospy

from sbpl_demos.srv import StateMachine

state = {'requested_object':'', 'ROMAN_STATE':'IDLE', 'PR2_STATE':'IDLE'}

# StateMachine.srv
#
# string command
# string request_key
# string request_value
# ---
# bool result
# string result_value

# command: 'Get', 'Set'
# request_key: 'requested_object', 'ROMAN_STATE', 'PR2_STATE'

# key: 'requested_object'
# value: '', '003_cracker_box', '006_mustard_bottle'
#
# key: 'ROMAN_STATE'
# value: 'IDLE', 'MOVE_ARM', 'DONE'
#
# key: 'PR2_STATE'
# value: 'IDLE', 'MOVE_ARM', 'MOVE_BASE', 'DONE'

def state_machine(request):
    global state

    if request.command == "Get":
        if request.request_key in state:
            result = True
            return (result, state[request.request_key])
        else:
            result = False
            return (result, '')

    if request.command == "Set":
        if not (request.request_key in state):
            rospy.logwarn("request_key '%s' is not in 'state' dictionary! Adding this as a new key...", request.request_key)
        state[request.request_key] = request.request_value
        result = True
        return (result, state[request.request_key])

if __name__ == "__main__":
    rospy.init_node('state_machine');
    service = rospy.Service('state_machine', StateMachine, state_machine)
    rospy.loginfo("/state_machine is now running...")
    rospy.spin()
