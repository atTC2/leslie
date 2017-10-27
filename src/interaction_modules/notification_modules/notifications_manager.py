#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import json
import beep_module
import email_module
import twitter_module


def manage_notification(data):
    # The first 6 characters of a ROS message are 'data: ', which
    # can be removed so we can parse the rest of the message into JSON
    data = json.loads(data.data)

    if data['send_beep']:
        beep_module.notify(data)
    if data['send_email']:
        email_module.notify(data)
    if data['send_tweet']:
        twitter_module.notify(data)


def start_listen():
    rospy.init_node('notifications_node', anonymous=True)
    rospy.Subscriber('notifier', String, manage_notification)
    rospy.spin()


if __name__ == '__main__':
    start_listen()
