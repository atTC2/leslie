#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy
import json

pub = rospy.Publisher('/state', String, queue_size=10)

if __name__ == "__main__":
    rospy.init_node('tester')
    
    state_data = {'id':'FOLLOW_PERP', 'data': {
        'angle': numpy.random.randint(-45, 45),
        'distance': 1
    }}
    pub.publish(json.dumps(state_data))
    print 'PUBLISHED FUK YEHHHHHHH, %s' % state_data['data']['angle']
    #rospy.sleep(10)
