import rospy
from std_msgs.msg import String


class YesNoListener:

    def __init__(self):
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.voice_report)

        # A mapping from keywords or phrases to commands
        self.commands = ['yes', 'no']

        rospy.loginfo("incident_report - ready to receive voice commands")

        self.pub = rospy.Publisher('/action', String, queue_size=1)

        self.callback = None

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
                tmp_callback = self.callback
                self.callback = None
                tmp_callback(True)
            elif command.find('no') != -1:
                tmp_callback = self.callback
                self.callback = None
                tmp_callback(False)
        else:  # command not found
            print 'command not recognised'

