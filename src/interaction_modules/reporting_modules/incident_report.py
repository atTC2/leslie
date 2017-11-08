import pyttsx
import rospy
import json
from std_msgs.msg import String
from state_machine import actions

engine = pyttsx.init()  # speech engine


class IncidentReport:
    # define the constructor of the class

    def __init__(self):

        speak("would you like an incident report via email")

        # initialize the ROS node with a name incident_report
        rospy.init_node('incident_report')

        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.voice_report)

        # A mapping from keywords or phrases to commands
        self.commands = ['yes', 'no']

        rospy.loginfo("Ready to receive voice commands")

        self.pub = rospy.Publisher('/action', String, queue_size=1)

        self.incident_end = True

    def voice_report(self, msg):

        if not self.incident_end:
            # Don't run now
            return

        command = msg.data

        report = -1
        if command in self.commands:
            if command.find('yes') != -1:
                speak("sending report")
                report = 0
            elif command.find('no') != -1:
                speak("okay i will not send a report")
                report = 1

            self.incident_end = False
            action = {}
            action['id'] = actions.CALLED_OVER
            action['data'] = {'reportID': report}
            self.pub.publish(json.dumps(action))
        else:  # command not found
            print ("command not recognised")


def speak(arg):
    engine.say(arg)
    engine.say("   ")
    engine.runAndWait()


if __name__ == "__main__":
    try:
        IncidentReport()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice terminated.")
