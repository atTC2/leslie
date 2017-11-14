#!/usr/bin/env python
import json

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from navstack_supervisor import client # use same client
from state_machine import states
from util_modules import utils_maths


if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


rospy.init_node('follow_perp_navstack')
rospy.Subscriber('/state', String, follow_callback, queue_size=10)


rospy.spin()
