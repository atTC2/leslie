#!/usr/bin/env python

import rospy

import fake_pub_base
from state_machine import actions, states


rospy.init_node('fake_pub_move_home', anonymous=True)

fake_pub_base.block_until_state(states.ALARM_REPORT)

fake_pub_base.publish(
    {
        'id': actions.ALARM_HANDLED,
        'data': None
    }
)
