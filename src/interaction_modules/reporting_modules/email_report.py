import json
from datetime import datetime
from functools import partial
from os.path import getctime
from sys import stderr

import rospy
from std_msgs.msg import String

from interaction_modules.email_util import create_email, send_email, attach_body, attach_file
from interaction_modules.yes_no_listener import YesNoListener
from state_machine import actions
from util_modules import speech_engine
from util_modules.people_info import get_user_info

yes_no_listener = YesNoListener()
pub = rospy.Publisher('/action', String, queue_size=10)


def ask_send_report_email(name, file_path):
    """
    Ask the user if they wish to have an email report sent to them (if they have an address configured) and sends if
    confirmed
    :param name: The name of whom the report is being sent to
    :param file_path: The path to the video file
    :type name: str
    :type file_path: str
    """
    # Check if they have an email address
    user_info = get_user_info(name)
    if 'email_address' not in user_info:
        # No email address
        handled_alarm()
        return

    # Give the incident reporter a new callback
    speech_engine.say("would you like an incident report via email")
    yes_no_listener.callback = partial(real_send_email, name, user_info['email_address'], file_path)


def real_send_email(name, recipient_address, file_path, would_like_email):
    """
    Actually send a report email including a video recording (if user confirmed) and forward state
    :param name: The name of whom the report is being sent to
    :param recipient_address: The email address of the user
    :param file_path: The path to the video file
    :param would_like_email: Whether to send an email
    :type name: str
    :type recipient_address: str
    :type file_path: str
    :type would_like_email: bool
    """
    if not would_like_email:
        speech_engine.say("okay, i will not send you the report")
        handled_alarm()
        return

    speech_engine.say("sending report")

    # Get the incident time
    try:
        incident_time = str(datetime.fromtimestamp(getctime(file_path)))
    except OSError:
        print >> stderr, 'File not available:\n', file_path
        return

    # Create the message
    msg = create_email(recipient_address, 'PIPS Incident @ ' + incident_time)

    # Attach the text
    text_content = 'Hello ' + name + '.\r\n\r\n'
    text_content += 'Attached is a video recording of a thief detected at ' + incident_time + '.\r\n\r\n'
    text_content += 'Kind regards,\r\nLeslie (Personal Item Protection System)'
    attach_body(msg, text_content)

    # Attached the video
    if not attach_file(msg, file_path):
        # Attaching the video failed (and was logged)
        return

    # Send the email
    send_email(recipient_address, msg)
    handled_alarm()


def handled_alarm():
    """
    Forward state
    """
    pub.publish(json.dumps({'id': actions.ALARM_HANDLED, 'data': {}}))
