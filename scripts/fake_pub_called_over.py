#!/usr/bin/env python

import rospy
import fake_pub_base
from state_machine import actions, states

rospy.init_node('fake_pub_called_over', anonymous=True)

# Command line parameters.
owner = rospy.get_param('~owner')  # type: str
if type(owner) is not str:
    raise Exception("'_owner' must be of type 'str'")
owner = owner.capitalize()

friend = rospy.get_param('~friend', None)  # type: str or None
if friend is not None:
    if type(friend) is not str:
        raise Exception("'_friend' must be of type 'str'")
    friend = friend.capitalize()

table = rospy.get_param('~table')  # type: int
if type(table) is not int:
    raise Exception("'_table' must be of type 'int'")

fake_pub_base.block_until_state(states.AT_HOME)

# Publish straight away without checking the state, since the initial state isn't published.
fake_pub_base.publish(
    {
        'id': actions.CALLED_OVER,
        'data': {
            'tableID': table,
            'current_owner': owner,
            'friend': friend
        }
    }
)
