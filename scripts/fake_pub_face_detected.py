#!/usr/bin/env python

import rospy

import fake_pub_base
from state_machine import actions


rospy.init_node('fake_pub_ready_to_lock', anonymous=True)

# Command line parameters.
owner = rospy.get_param('~owner')  # type: str
if type(owner) is not str:
    raise Exception("'_owner' must be of type 'str'")
owner = owner.capitalize()

fake_pub_base.publish(
    {
        'id': actions.FACE_DETECTED,
        'data': {
            'current_owner': owner
        }
    }
)
