#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from state_machine import actions, states, state_util
from get_table import GetTable
from get_friend import GetFriend

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


owner = None
friend = None
get_table = GetTable()
get_friend = GetFriend()


def got_friend(selected_friend):
    """
    Takes a selected friend, stores tha information, and starts the process of listing for table instructions
    :param selected_friend: The friend selected, or None if no friend selected
    :type selected_friend: str
    """
    global friend
    get_friend.stop_listening()
    friend = selected_friend
    get_table.listen()


def got_table(selected_table):
    """
    Takes a selected table and publishes the competed action, with friend information previously gathered
    :param selected_table: The selected table by the user
    :type selected_table: int
    """
    global owner, friend
    get_table.stop_listening()
    action = {
        'id': actions.CALLED_OVER,
        'data': {
            'tableID': selected_table,
            'current_owner': owner,
            'friend': friend
        }
    }
    pub.publish(json.dumps(action))


def state_callback(state_msg):
    """
    Handles stage changes
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    :return:
    """
    global get_friend, get_table, owner, friend
    state_json = json.loads(state_msg.data)
    state_id = state_json['id']

    # Ensure nothing is listening and reset values
    get_table.stop_listening()
    get_friend.stop_listening()
    owner = None
    friend = None

    # Do things dependant upon the new state
    if state_id == states.LISTENING_FOR_TABLE:
        owner = state_json['data']['current_owner']
        # Determine if they have already selected (or not selected) a friend (i.e. have rejected a table)
        if 'friend' not in state_json['data']:
            # Not before had the friend option
            get_friend.listen()
        else:
            # Already selected friend, so save it here and go straight to table selection
            friend = state_json['data']['friend']
            get_table.listen()


# Set callbacks
get_friend.callback = got_friend
get_table.callback = got_table

# ROS node stuff
rospy.init_node('voice_control')
pub = rospy.Publisher('/action', String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
state_util.prime_state_callback_with_starting_state(state_callback)
rospy.loginfo("voice_control - ready to receive voice commands")
rospy.spin()
