import json
import operator

import rospy
from std_msgs.msg import String


# Should not be accessed until after _block_until_operator_state.
state_id = None
state_data = None


def _state_callback(state_msg):
    """
    Method called every `/state` change.
    Updates the global variables `state_id` and `state_data`.
    :param state_msg: The state change message.
    :type state_msg: String
    """
    global state_id
    global state_data
    
    state_json = json.loads(state_msg.data)
    state_id = state_json['id']
    state_data = state_json['data']


rospy.Subscriber('/state', String, _state_callback, queue_size=10)

_PUB = rospy.Publisher('/action', String, queue_size=10)


def publish(action_dict):
    """
    Publishes and action to `/action`.
    This methods blocks until the state changes to confirm the published action worked.
    If the state machine is not in the correct state then it will not work.
    Therefore it is important to either only run when you're in the correct state or use `block_until_state`.
    :param action_dict: The action object to publish. (With fields "id" and "data")
    :type action_dict: dict
    """
    global state_id
    global _PUB
    locked_on_state = state_id
    
    action_dumped = json.dumps(action_dict)
    print 'Publishing', action_dumped
    _PUB.publish(action_dumped)
    
    block_until_not_state(locked_on_state)


def block_until_not_state(state):
    """
    Blocks until the state changes to anything but the state specified.
    :param state: The state Id to change away from.
    :type state: str
    """
    print 'Waiting for any state but', state
    _block_until_operator_state(operator.ne, state)


def block_until_state(state):
    """
    Blocks until the state changes to the state specified.
    :param state: The state Id to wait for.
    :type state: str
    """
    print 'Waiting for state', state
    _block_until_operator_state(operator.eq, state)


def _block_until_operator_state(op, state):
    """
    Blocks until the state changes to satisfy the operator specified when applied with the old state.
    :param op: The operator to check the new and old state with. The new state going into the left hand side.
    :param state: The old state to check against.
    :type op: (str, str) -> bool
    :type state: str
    """
    global state_id
    try:
        while not op(state_id, state):
            rospy.sleep(2)
        print 'New state', state_id
    except rospy.ROSInterruptException:
        print 'ROSInterruptException'
        exit(1)
