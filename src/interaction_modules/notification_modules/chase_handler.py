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
        speech_engine.say("It seems there was a false alarm, please be sure to unlock me with your face before approaching the table in future.")
        action['id'] = actions.ALARM_HANDLED
    elif state['id'] == states.LOST:
        speech_engine.say("Video recording of incident complete, returning to guard post.")
        action['id'] = actions.ALARM_HANDLED
    pub.publish(json.dumps(action))

rospy.init_node("chase_handler_node")
pub = rospy.Publisher("/action", String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
rospy.spin()
