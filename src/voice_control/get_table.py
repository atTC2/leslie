#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from util_modules import speech_engine


class GetTable:

    def __init__(self):
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.voice_command_callback)

        self.state = 1  # State of voice control
        self.selected_table = -1  # table selected by user [0-4]
        self.print_table = -1  # table selected by user plus 1 for speech [1-5]
        self.listening = False
        self.callback = None

    def listen(self):
        """
        Sets the listener to listening, speaks instructions to the user and ensures any important values are reset
        """
        speech_engine.say('which table would you like to go to')
        self.listening = True
        self.state = 1

    def stop_listening(self):
        """
        Stops the friend finder listening
        """
        self.listening = False

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
            print 'Processing table instruction...'
            self.selected_table = self.get_table(msg)

        elif self.state == 2:
            print 'Processing instruction confirmation for table', self.selected_table
            self.check_table(msg)

    def get_table(self, msg):
        """
        Finds a table inputted through voice
        :param msg: The message containing the voice input
        :type msg: std_msgs.msg.String
        """
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
            self.state = 2
            self.print_table = table + 1
            speech_engine.say("would you like to go to table " + str(self.print_table))
        # Re-ask the question if command not recognised

        return table

    def check_table(self, msg):
        """
        Verifies the correct table has been selected
        :param msg: The message containing the voice input
        :type msg: std_msgs.msg.String
        """
        # User response to verifying table selected
        yesno = msg.data

        # If table correct publish table number to state machine
        if yesno == 'yes':
            speech_engine.say('Going to table ' + str(self.print_table))
            self.callback(self.selected_table)
        # If table not correct ask again which table leslie should go to
        elif yesno == 'no':
            speech_engine.say('which table would you like to go to')
            self.state = 1
        # If user says yes nor no then re-ask the question
        else:
            print 'command not recognised'
