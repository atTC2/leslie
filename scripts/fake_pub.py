#!/usr/bin/env python

import json

import rospy
from std_msgs.msg import String

from state_machine import actions


if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


state_msg_received = False


def state_callback(_):
    global state_msg_received
    state_msg_received = True


rospy.init_node('fake_pub', anonymous=True)

rospy.Subscriber('/state', String, state_callback, queue_size=1)

pub = rospy.Publisher('/action', String, queue_size=1)

action = {
    'id': actions.CALLED_OVER,
    'data': {
        'tableID': 0
    }
}

rospy.sleep(2)
pub.publish(json.dumps(action))

try:
    while not state_msg_received:
        rospy.sleep(2)
except rospy.ROSInterruptException:
    pass
