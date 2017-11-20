#!/usr/bin/env python

import rospy

import fake_pub_base
from state_machine import actions, states


rospy.init_node('fake_pub_ready_to_lock', anonymous=True)

fake_pub_base.block_until_state(states.AT_TABLE)

fake_pub_base.publish(
    {
        'id': actions.READY_TO_LOCK,
        'data': fake_pub_base.state_data
    }
)
