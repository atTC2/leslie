#!/usr/bin/env python

import json

import rospy
from std_msgs.msg import String

import beep_module
import email_module
import twitter_module
from state_machine import states
from util_modules.people_info import get_user_info

if __name__ != '__main__':
    from sys import stderr

    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


def manage_notification(notify_owner):
    """
    Sends alarm notifications to the owner and plays out a message
    :param notify_owner: The identifier of the owner
    :type notify_owner: String
    """
    # Get someones info from json
    data = get_user_info(notify_owner)

    # Beep
    beep_module.notify()

    # Twitter
    if 'twitter' in data:
        twitter_module.notify(data['twitter'])
    # Email
    if 'email_address' in data:
        email_module.notify(notify_owner, data['email_address'])


def state_callback(state_msg):
    state_json = json.loads(state_msg.data)
    state_id = state_json['id']
    state_data = state_json['data']
    
    if state_id != states.ALARM:
        return
    
    owner_name = state_data['current_owner']
    manage_notification(owner_name)
    

rospy.init_node('notifications_manager')
rospy.Subscriber('/state', String, state_callback, queue_size=1)
rospy.spin()
