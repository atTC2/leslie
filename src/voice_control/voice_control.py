#!/usr/bin/env python

import pyttsx
import rospy
import json
from std_msgs.msg import String
from state_machine import actions, states


if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


engine = pyttsx.init()  # speech engine
state = 1  # State of voice control
selected_table = -1  # table selected by user [0-4]
print_table = -1  # table selected by user plus 1 for speech [1-5]


class VoiceControl:
    # define the constructor of the class
    def __init__(self):

        speak("which table would you like to go to")

        # initialize the ROS node with a name voice_teleop
        rospy.init_node('voice_control')

        rospy.Subscriber('/state', String, self.state_call_back, queue_size=1)

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.voice_command_callback)

        rospy.loginfo("Ready to receive voice commands")

        self.pub = rospy.Publisher('/action', String, queue_size=1)

        self.at_home = True

    # choose correct question and voice response for current state
    def voice_command_callback(self, msg):

        global state
        global selected_table

        if not self.at_home:
            # Don't run now
            return

        if state == 1:

            print("State 1")
            selected_table = get_table(msg)

        elif state == 2:
            print ("State 2")
            check_table(self, msg)

    def state_call_back(self, data):

        global state

        state_json = json.loads(data.data)
        state_id = state_json['id']
        if state_id == states.WAIT_FOR_INSTRUCTION:
            state = 1
            self.at_home = True
        else:
            self.at_home = False


# Leslie speaks sentence - args
def speak(arg):
    engine.say(arg)
    engine.say("   ")
    engine.runAndWait()


# Allow user to select table
def get_table(msg):

    global state
    global print_table

    # Get the motion command from the recognized phrase
    table = -1
    command = msg.data
    # User selects a table with voice control
    if command.find('one') != -1:
        table = 0
    elif command.find('two') != -1:
        table = 1
    elif command.find('three') != -1:
        table = 2
    elif command.find('four') != -1:
        table = 3
    elif command.find('five') != -1:
        table = 4
    else:  # command not found
        print ("Command not recognised")

    # If table has been selected check it is correct
    if table != -1:
        print("change state")
        state = 2
        print_table = table + 1
        speak("would you like to go to table " + str(print_table))
    # Re-ask the question if command not recognised

    return table


# Allow user to verify table
def check_table(self, msg):
    global selected_table
    global print_table
    global state

    # User response to verifying table selected
    yesno = msg.data

    # If table correct publish table number to state machine
    if yesno == 'yes':
        self.at_home = False
        speak("Going to table " + str(print_table))
        action = {}
        action['id'] = actions.CALLED_OVER
        action['data'] = {'tableID': selected_table}
        self.pub.publish(json.dumps(action))
    # If table not correct ask again which table leslie should go to
    elif yesno == 'no':
        speak("which table would you like to go to")
        state = 1
    # If user says yes nor no then re-ask the question
    else:
        print("command not recognised")


try:
    VoiceControl()
    rospy.spin()
except rospy.ROSInterruptException:
    rospy.loginfo("Voice navigation terminated.")
