import rospy
from std_msgs.msg import String
import sys
import json


class StateIDs(object):
    """
    Possible states the system can be in
    """
    WAIT_FOR_INSTRUCTION = "Waiting for instruction"
    MOVE_TO_TABLE = "Moving to table"
    AT_TABLE = "At table"
    LOCKING = "Locking"
    LOCKED_AND_WAITING = "Locked and waiting"
    ALARM = "ALARM!"
    ALARM_REPORT = "Reporting on alarm"
    MOVE_TO_HOME = "Moving to home"


class ActionIDs(object):
    """
    Actions that can be taken
    """
    CALLED_OVER = "Got instruction"
    ARRIVED = "Arrived"
    GOT_FACE = "Got face(s)"
    READY_TO_LOCK = "Read to lock"
    MOVEMENT_DETECTED = "Movement detected!"
    ALARM_HANDLED = "Alarm handled (phew!)"
    FACE_RECOGNISED = "Face recognised"


# Mapping of state + action = state
state_machine = dict()
state_machine[(StateIDs.WAIT_FOR_INSTRUCTION, ActionIDs.CALLED_OVER)] = StateIDs.MOVE_TO_TABLE
state_machine[(StateIDs.MOVE_TO_TABLE, ActionIDs.ARRIVED)] = StateIDs.AT_TABLE
state_machine[(StateIDs.AT_TABLE, ActionIDs.GOT_FACE)] = StateIDs.LOCKING
state_machine[(StateIDs.LOCKING, ActionIDs.READY_TO_LOCK)] = StateIDs.LOCKED_AND_WAITING
state_machine[(StateIDs.LOCKED_AND_WAITING, ActionIDs.MOVEMENT_DETECTED)] = StateIDs.ALARM
state_machine[(StateIDs.ALARM, ActionIDs.FACE_RECOGNISED)] = StateIDs.ALARM_REPORT
state_machine[(StateIDs.ALARM_REPORT, ActionIDs.ALARM_HANDLED)] = StateIDs.MOVE_TO_HOME
state_machine[(StateIDs.LOCKED_AND_WAITING, ActionIDs.FACE_RECOGNISED)] = StateIDs.MOVE_TO_HOME
state_machine[(StateIDs.MOVE_TO_HOME, ActionIDs.ARRIVED)] = StateIDs.WAIT_FOR_INSTRUCTION

# Always start on WAIT FOR INSTRUCTION
current_state_id = StateIDs.WAIT_FOR_INSTRUCTION


def publish_state(data):
    """
    Publishes the current state to topic 'state'
    """
    global current_state_id

    state = {}
    state['id'] = current_state_id
    state['data'] = data

    pub.publish(json.dumps(state))


def action_callback(action_msg):
    """
    Sets a new state given the current state allows for the given action to be taken
    :param action_msg: The The message received from the 'action' topic
    :type action_msg: std_msgs.String
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
pub = rospy.Publisher('/state', String, queue_size=1)
rospy.Subscriber('/action', String, action_callback, queue_size=1)
