#!/usr/bin/env python
import json
import rospy
from std_msgs.msg import String
from notification_builder import NotificationBuilder


if __name__ == '__main__':
    try:
        pub = rospy.Publisher('notifier', String, queue_size=10)
        rospy.init_node('notifications_node', anonymous=True)

        builder = NotificationBuilder('Seb')
        builder.send_email('leslietherobot.aka.pips@gmail.com',
                           'You\'re the best!')
        builder.send_beep('You\'re the best!')
        builder.send_tweet('Seb, you\'re the best!')

        json_string = json.dumps(builder.data)
        pub.publish(json_string)
    except rospy.ROSInterruptException:
        pass
