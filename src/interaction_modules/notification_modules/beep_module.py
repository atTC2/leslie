import rospy
from state_machine import states, state_util
from util_modules import speech_engine

state_id = state_util.get_start_state()


def notify():
    """
    Says alarm repeatedly!
    """
    while state_id == states.ALARM:
        speech_engine.say('beep beep bloop blob ALARM!')
        rospy.sleep(1)
