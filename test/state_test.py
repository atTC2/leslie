#!/usr/bin/env python

import unittest
import rostest
import rospy
from std_msgs.msg import String
from state_machine.state_machine import States, Actions

# Had to make these global so we could write tests
callback_called = 0
result_state = None


class StateMachineTestClass(unittest.TestCase):
    def test_change_state(self):
        """
        Test to ensure the 'waiting' state combined with the published action 'called over' causes a new published
        state of 'move to table'
        """
        global callback_called
        global result_state
        callback_called = 0

        def callback(state_msg):
            global callback_called
            global result_state
            callback_called += 1
            result_state = state_msg.data

        rospy.Subscriber('/state', String, callback, queue_size=1)
        pub = rospy.Publisher('/action', String, queue_size=1)
        # Ensures the publisher and subscriber have had time to register
        rospy.sleep(1)
        pub.publish(Actions.CALLED_OVER)
        rospy.sleep(1)
        self.assertEqual(callback_called, 1,
                         'Callback not called correct amount of times (' + str(callback_called) + ')')
        self.assertEqual(result_state, States.MOVE_TO_TABLE, 'State not updated correctly (' + result_state + ')')


if __name__ == '__main__':
    rospy.init_node('state_test')
    rostest.rosrun('leslie', 'state_test', StateMachineTestClass)
