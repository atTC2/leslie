#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import json
from states import *
from actions import *


if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


# Mapping of state + action = state
'''
state_machine = {
    # State                Action           -> State
    (WAIT_FOR_INSTRUCTION, CALLED_OVER):       MOVE_TO_TABLE,
    (MOVE_TO_TABLE,        ARRIVED):           AT_TABLE,
    (AT_TABLE,             GOT_FACE):          LOCKING,
    (LOCKING,              READY_TO_LOCK):     LOCKED_AND_WAITING,
    (LOCKED_AND_WAITING,   MOVEMENT_DETECTED): ALARM,
    (ALARM,                FACE_RECOGNISED):   ALARM_REPORT,
    (ALARM_REPORT,         ALARM_HANDLED):     MOVE_TO_HOME,
    (LOCKED_AND_WAITING,   FACE_RECOGNISED):   MOVE_TO_HOME,
    (MOVE_TO_HOME,         ARRIVED):           WAIT_FOR_INSTRUCTION
}
'''

state_machine = {
    # State                Action           -> State
    (WAIT_FOR_INSTRUCTION, CALLED_OVER):       MOVE_TO_TABLE,
    (MOVE_TO_TABLE,        ARRIVED):           AT_TABLE,
    (AT_TABLE,             GOT_FACE):          LOCKING,
    (LOCKING,              READY_TO_LOCK):     LOCKED_AND_WAITING,
    (LOCKED_AND_WAITING,   MOVEMENT_DETECTED): ALARM,
    (ALARM,                CAUGHT_THIEF):      CAUGHT,
    (ALARM,                FACE_RECOGNISED):   ACCIDENT,
    (CAUGHT,               FACE_RECOGNISED):   ACCIDENT,
    (CAUGHT,               RECOG_TIMEOUT):     THIEF,
    (ACCIDENT,             ALARM_HANDLED):     MOVE_TO_HOME,
    (THIEF,                ALARM_HANDLED):     MOVE_TO_HOME,
    (LOCKED_AND_WAITING,   FACE_RECOGNISED):   MOVE_TO_HOME,
    (MOVE_TO_HOME,         ARRIVED):           WAIT_FOR_INSTRUCTION,
    (ALARM,                LOST_THIEF):        LOST,
    (LOST,                 ALARM_HANDLED):     MOVE_TO_HOME
}

# Always start on WAIT FOR INSTRUCTION
current_state_id = AT_TABLE


def publish_state(data):
    """
    Publishes the current state to topic '/state'
    :param data: The JSON data to pass in the '/state' topic.
    :type data: dict(str, T)
    """
    global current_state_id

    state = {}
    state['id'] = current_state_id
    state['data'] = data

    print 'State:', state
    pub.publish(json.dumps(state))


def action_callback(action_msg):
    """
    Sets a new state given the current state allows for the given action to be taken
    :param action_msg: The message received from the '/action' topic
    :type action_msg: std_msgs.msg.String
    """
    global current_state_id

    action_taken = json.loads(action_msg.data)
    action_id = action_taken['id']
    action_data = action_taken['data']

    try:
        current_state_id = state_machine[(current_state_id, action_id)]
        publish_state(action_data)
    except KeyError:
        print >> sys.stderr, 'Error updating state! Current state:', current_state_id, 'Action:', action_id


# Setup publisher for states and subscriber for actions
rospy.init_node('state_machine')
pub = rospy.Publisher('/state', String, queue_size=10)
rospy.Subscriber('/action', String, action_callback, queue_size=10)
rospy.spin()
