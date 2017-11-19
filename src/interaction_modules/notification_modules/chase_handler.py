#!/usr/bin/env python
from std_msgs.msg import String
from state_machine import states, actions
import json
import rospy
from util_modules import speech_engine

def state_callback(state_msg):
    state = json.loads(state_msg.data)
    action = {}
    action['data'] = {}
    if state['id'] == states.ACCIDENT:
        action['id'] = actions.ALARM_HANDLED
    elif state['id'] == states.CAUGHT:
        speech_engine.say("Please present your face for identification.")
    elif state['id'] == states.LOST:
        action['id'] = actions.ALARM_HANDLED
    elif state['id'] == states.THIEF:
        speech_engine.say("You are not recognised as the owner. Return what you have stolen, thief. I am sending a video of this encounter to the owner.")
        action['id'] = actions.ALARM_HANDLED
    pub.publish(json.dumps(action))

rospy.init_node("chase_handler_node")
pub = rospy.Publisher("/action", String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
rospy.spin()
