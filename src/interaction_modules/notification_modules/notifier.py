import json
import rospy
from std_msgs.msg import String

def notify(recipient, email, message, send_beep, send_sms, send_email):
    pub = rospy.Publisher('notifier', String, queue_size=10)
    rospy.init_node('notifications_node', anonymous=True)

    data = {}
    data['recipient'] = recipient
    data['message'] = message
    data['email'] = email
    data['send_beep'] = send_beep
    data['send_sms'] = send_sms
    data['send_email'] = send_email
    json_string = json.dumps(data)

    pub.publish(json_string)

if __name__ == '__main__':
    try:
        notify('Seb', 'leslietherobot.aka.pips@gmail.com', 'You\'re the best!', True, False, True)
    except rospy.ROSInterruptException:
        pass
