#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
from tweepy import TweepError, StreamListener, Stream
from state_machine import actions, states
from util_modules import people_info, twitter_api

state_id = states.AT_HOME

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


class TwitterControl(StreamListener):

    def on_status(self, status):
        """
        When a status is received, pass to the status handler
        :param status: The received status
        :type status: tweepy.Status
        :return: True, as it's ok
        :rtype: bool
        """
        process_tweet(status)
        return True

    def on_error(self, status_code):
        """
        Handles error codes. This closes twitter_control
        :param status_code: The code describing the error
        :type status_code: int
        :return: False, as it's gone wrong
        :rtype: bool
        """
        print 'Tweepy on_error', status_code
        return False


def process_tweet(tweet):
    """
    If the tweet contains a table reference, it shall find the table number and call move_to_table_handler
    If the tweet isn't understood (e.g. table without a number) it is ignored
    :param tweet: The tweet object
    :type tweet: tweepy.models.Status
    :return:
    """
    table_str = 'table'
    text = tweet.text.lower().split()

    if table_str not in text or text.index(table_str) == len(text) - 1:
        # Not a table tweet (or one we can process), we don't want this
        return

    # Find the user information
    handle = tweet.author.screen_name
    user = people_info.get_user_from_twitter_handle(handle)

    if user is None:
        # No user found
        return

    # Find the table information in the tweet
    table_index = text.index(table_str)
    table_number = text[table_index + 1]

    # Find friend
    friend = None
    for token in text:
        # Find tagged twitter handles
        if token.startswith('@'):
            friend = people_info.get_user_from_twitter_handle(token[1:])
            # Ensure we haven't accidentally picked the requester
            if friend == user:
                friend = None
        # If we've found a friend, stop looping
        if friend is not None:
            break

    if table_number in ['1', 'one']:
        move_to_table_handler(0, user, handle, friend, tweet.id)
    elif table_number in ['2', 'two']:
        move_to_table_handler(1, user, handle, friend, tweet.id)
    elif table_number in ['3', 'three']:
        move_to_table_handler(2, user, handle, friend, tweet.id)
    elif table_number in ['4', 'four']:
        move_to_table_handler(3, user, handle, friend, tweet.id)
    elif table_number in ['5', 'five']:
        move_to_table_handler(4, user, handle, friend, tweet.id)


def move_to_table_handler(table_number, user, handle, friend, tweet_id):
    """
    Send the system to the table number (if the state allows)
    :param table_number: The table number to go to
    :param user: The user (seen in people.json) who is requesting Leslie
    :param handle: The twitter handle of the user
    :param friend: A friend that can unlock the system
    :param tweet_id: The ID of the tweet (so it can be replied to)
    :type table_number: int
    :type user: str
    :type handle: str
    :type friend: str
    :type tweet_id: int
    """
    global state_id

    # Check to see if we can do this
    if state_id == states.AT_HOME:
        # In a state to move to table
        action = {
            'id': actions.CALLED_OVER,
            'data': {
                'tableID': table_number,
                'current_owner': user,
                'friend': friend
            }
        }
        pub.publish(json.dumps(action))
        tweet_reply("@" + handle + " Heading over! See you there.", tweet_id)
    else:
        # Not able to move to table
        tweet_reply("@" + handle + " Sorry, I'm busy right now.", tweet_id)


def tweet_reply(message, tweet_id):
    """
    Reply to the request tweet
    :param message: The message to send
    :param tweet_id: The ID of the tweet to reply to
    :type message: str
    :type tweet_id: int
    :return:
    """
    try:
        twitter_api.api.update_status(message, tweet_id)
    except TweepError:
        print 'Unable to send reply tweet'


def state_callback(state_msg):
    """
    Processes updates to state information
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global state_id

    state_id = json.loads(state_msg.data)['id']


# Setup ROS things
rospy.init_node("twitter_control")
rospy.Subscriber('/state', String, state_callback, queue_size=10)
pub = rospy.Publisher('/action', String, queue_size=10)

# Setup Twitter listener
twitter_control = TwitterControl()
twitter_stream = Stream(auth=twitter_api.api.auth, listener=twitter_control)
twitter_stream.filter(track=['@LeslieTheRobot'])
