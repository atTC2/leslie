import rospy
from std_msgs.msg import String
from util_modules import speech_engine


class IncidentReport:

    def __init__(self):
        # initialize the ROS node with a name incident_report
        # rospy.init_node('incident_report')

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.voice_report)

        # A mapping from keywords or phrases to commands
        self.commands = ['yes', 'no']

        rospy.loginfo("incident_report - ready to receive voice commands")

        self.pub = rospy.Publisher('/action', String, queue_size=1)

        self.callback = None

    def prompt_email_confirmation(self, callback):
        """
        Set the call back to email sender and ask user for input
        :param callback: The callback function to call
        :type callback: (bool) -> None
        """
        speech_engine.say("would you like an incident report via email")
        self.callback = callback

    def voice_report(self, msg):
        """
        Check received speech for confirmation to email report
        :param msg: User speech
        :type msg: String
        """
        if self.callback is None:
            # Don't run now
            return

        command = msg.data

        if command in self.commands:
            if command.find('yes') != -1:
                speech_engine.say("sending report")
                self.callback(True)
            elif command.find('no') != -1:
                speech_engine.say("okay i will not send a report")
                self.callback(False)

            self.callback = None
        else:  # command not found
            print ("command not recognised")

