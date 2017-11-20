import json

from std_msgs.msg import String

from util_modules import config_access


def get_start_state():
    """
    Gets the initial state
    :return: The initial state
    :rtype: str
    """
    return config_access.get_config(config_access.KEY_STARTING_STATE)['id']


def send_state_obj_to_state_callback(state_callback, state_obj):
    """
    Sends an object (dict) representing a state message to a state callback as a String.
    :param state_callback: The callback to pass the message to.
    :param state_obj: The state object (dict) with fields "id" and "data".
    :type state_callback: (String) -> T
    :type state_obj: dict
    """
    state_callback(String(json.dumps(state_obj)))


def prime_state_callback_with_starting_state(state_callback):
    """
    Sends the default state message to a state callback.
    :param state_callback: The callback to pass the message to.
    :type state_callback: (String) -> T
    """
    state_obj = config_access.get_config(config_access.KEY_STARTING_STATE)
    send_state_obj_to_state_callback(state_callback, state_obj)
