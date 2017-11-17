#!/usr/bin/env python

import rospy

import fake_pub_base
from state_machine import actions, states


rospy.init_node('fake_pub_movement_detected', anonymous=True)

fake_pub_base.block_until_state(states.LOCKED_AND_WAITING)

fake_pub_base.publish(
    {
        'id': actions.MOVEMENT_DETECTED,
        'data': fake_pub_base.state_data
    }
)
