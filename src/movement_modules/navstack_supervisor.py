#!/usr/bin/env python

import json

import actionlib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.srv import GetMap
from std_msgs.msg import String

from state_machine import states, actions

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


#  --- Get info about static map from map_server

#  waits for map_server, which should be launched beforehand
rospy.wait_for_service('static_map')
map_getter = rospy.ServiceProxy('static_map', GetMap)
meta_data = map_getter().map.info
map_origin = meta_data.origin
map_resolution = meta_data.resolution

#  --- --- --- ---

#  --- Create the Pose for leslie at each table

#  The orientation for leslie to face the table, taken from Rviz
table_orientation = Quaternion(0.0, 0.0, -0.200032655658, 0.979789230738)
table = []
for i in range(5):
    p = Pose()
    p.orientation = table_orientation
    table.append(p)

table[0].position = Point(-2.03, -4.8, 0)
table[1].position = Point(-1.09, -1.957, 0)
table[2].position = Point(0.193, 0.385, 0)
table[3].position = Point(1.221, 3.015, 0)
table[4].position = Point(2.282, 5.741, 0)

#  --- --- --- --- --- ---

#  create action client, interface with navstack via client-server model
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


def state_callback(state_msg):
    """
    Runs if the current state is to MOVE_TO_TABLE.
    Launches the move_base launch file, which includes the navstack and amcl.
    sends the navstack a goal
    which was included in the message published with the /state topic,
    then waits for robot to reach that goal.

    :param state_msg: The current state of the robot's State Machine and data about table
    :type state_msg: std_msgs.msg.String
    """
    global client
    state_json = json.loads(state_msg.data)
    if state_json['id'] != states.MOVE_TO_TABLE:
        return

    client.wait_for_server()  # blocks indefinitely
    #  creates goal, send to navstack server and waits for navstack to run
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.pose = table[state_json['data']['tableID']]
    print str(goal.target_pose.pose)
    client.send_goal_and_wait(goal)
    print 'done waiting'

    #  after navstack completes, get its 'state' and check if reached goal
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print 'successfully reached goal'
        action_data = {'id': actions.ARRIVED, 'data': ''}
        state_pub.publish(String(json.dumps(action_data)))
    else:
        print 'fail to reach goal'
        # TODO: Return home


def goal_callback(goal):
    """
    Prints any received goal to the screen.
    Can be quite useful for debugging.

    :param goal: The goal of the navstack pathfinding
    :type goal: PoseStamped
    """
    print str(goal)


rospy.init_node('navstack_supervisor')
state_pub = rospy.Publisher('/action', String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)

# TESTING
# state_data = {'id': states.MOVE_TO_TABLE, 'data': {'tableID': 1}}
# state_callback(String(json.dumps(state_data)))

rospy.spin()
