#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from util_modules import speech_engine
from util_modules.people_info import is_user_in_system


class GetFriend:

    def __init__(self):
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.voice_command_callback)

        self.state = 1  # State of getting friend
        self.selected_friend = None
        self.listening = False
        self.callback = None

    def listen(self):
        """
        Sets the listener to listening, speaks instructions to the user and ensures any important values are reset
        """
        speech_engine.say('would you like to allow a friend to unlock this device?')
        speech_engine.say('either say their name, or no if you would not like to add someone')
        self.listening = True
        self.state = 1

    def stop_listening(self):
        """
        Stops the friend finder listening
        """
        self.listening = False

    # choose correct question and voice response for current state
    def voice_command_callback(self, msg):
        """
        Handles incoming voice input
        :param msg: The message containing the voice input
        :type msg: std_msgs.msg.String
        """
        if not self.listening:
            # Don't run now
            return

        if self.state == 1:
            print "Getting friend's name..."
            self.get_friend(msg)

        elif self.state == 2:
            print 'Checking selected friend', self.selected_friend
            self.check_friend(msg)

    def get_friend(self, msg):
        """
        Finds a name (valid in people.json) spoke by the user, or 'no' if they don't wish to have a friend selected
        :param msg: The message containing the voice input
        :type msg: std_msgs.msg.String
        """
        # Get the motion command from the recognized phrase
        command = msg.data
        # User selects a friend with voice control
        if command == 'no':
            self.callback(None)
        elif is_user_in_system(command.capitalize()):
            # command is user name
            self.selected_friend = command.capitalize()
            self.state = 2
            print 'checking friend state'
            speech_engine.say('are you sure you want to add ' + self.selected_friend)
        else:
            print 'command not recognised / user not found'

    def check_friend(self, msg):
        """
        Verifies with the user if the selected friend is correct
        :param msg: The message containing the voice input
        :type msg: std_msgs.msg.String
        """
        # User response to verifying friend selected
        yesno = msg.data

        # If friend correct publish friend number to state machine
        if yesno == 'yes':
            speech_engine.say("Added friend " + self.selected_friend)
            self.callback(self.selected_friend)
        elif yesno == 'no':
            self.listen()
        # If user says yes nor no then re-ask the question
        else:
            print 'command not recognised'
