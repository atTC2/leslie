import rospy
from std_msgs.msg import String
import sys


class States(object):
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


class Actions(object):
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
state_machine[(States.WAIT_FOR_INSTRUCTION, Actions.CALLED_OVER)] = States.MOVE_TO_TABLE
state_machine[(States.MOVE_TO_TABLE, Actions.ARRIVED)] = States.AT_TABLE
state_machine[(States.AT_TABLE, Actions.GOT_FACE)] = States.LOCKING
state_machine[(States.LOCKING, Actions.READY_TO_LOCK)] = States.LOCKED_AND_WAITING
state_machine[(States.LOCKED_AND_WAITING, Actions.MOVEMENT_DETECTED)] = States.ALARM
state_machine[(States.ALARM, Actions.FACE_RECOGNISED)] = States.ALARM_REPORT
state_machine[(States.ALARM_REPORT, Actions.ALARM_HANDLED)] = States.MOVE_TO_HOME
state_machine[(States.LOCKED_AND_WAITING, Actions.FACE_RECOGNISED)] = States.MOVE_TO_HOME
state_machine[(States.MOVE_TO_HOME, Actions.ARRIVED)] = States.WAIT_FOR_INSTRUCTION

# Always start on WAIT FOR INSTRUCTION
current_state = States.WAIT_FOR_INSTRUCTION


def publish_state():
    """
    Publishes the current state to topic 'state'
    """
    global current_state

    pub.publish(current_state)


def action_callback(action_msg):
    """
    Sets a new state given the current state allows for the given action to be taken
    :param action_msg: The The message received from the 'action' topic
    :type action_msg: std_msgs.String
    """
    global current_state

    action = action_msg.data
    try:
        current_state = state_machine[(current_state, action)]
        publish_state()
    except KeyError:
        print >> sys.stderr, 'Error updating state! Current state:', current_state, 'Action:', action


# Setup publisher for states and subscriber for actions
pub = rospy.Publisher('/state', String, queue_size=1)
rospy.Subscriber('/action', String, action_callback, queue_size=1)
