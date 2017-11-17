#!/usr/bin/env python

import rospy

import fake_pub_base
from state_machine import actions, states


rospy.init_node('fake_pub_face_recognised', anonymous=True)

# Command line parameters.
on_alarm = rospy.get_param('~on_alarm', False)  # type: bool
if type(on_alarm) is not bool:
    raise Exception("'_on_alarm' must be of type 'bool'")

fake_pub_base.block_until_state(states.ALARM if on_alarm else states.LOCKED_AND_WAITING)

fake_pub_base.publish(
    {
        'id': actions.FACE_RECOGNISED,
        'data': {
            'notify_owner': fake_pub_base.state_data['current_owner']
        }
    }
)
