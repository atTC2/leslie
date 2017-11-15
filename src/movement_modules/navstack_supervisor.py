#!/usr/bin/env python

import json

import actionlib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.srv import GetMap
from std_msgs.msg import String
# FOR TESTING: If we're trying relocalisation if we cannot reach the goal
# from std_srvs.srv import Empty

from interaction_modules.yes_no_listener import YesNoListener
from state_machine import states, actions
from util_modules import speech_engine

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

# positions are taken from rviz
table[0].position = Point(-1.8060284138, -4.99175453186, 0)
table[1].position = Point(-1.2, -2.0, 0)
table[2].position = Point(-0.04291536808, 0.249539896846, 0)
table[3].position = Point(1.05023245811, 2.81623911858, 0)
table[4].position = Point(2.22847919464, 5.56012153625, 0)

#  --- --- --- --- --- ---

home_pose = Pose()
home_pose.position = Point(-1.38991832733, -9.28993225098, 0)
home_pose.orientation = Quaternion(0, 0, 0.982110753886, 0.188304187691)

#  create action client, interface with navstack via client-server model
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

yes_no_listener = YesNoListener()


def state_callback(state_msg):
    """
    Runs if the current state is to MOVE_TO_TABLE or MOVE_TO_HOME.
    Sends the navstack a goal, which is either a table or home where
    table was included in the message published to the /state topic,
    then waits for robot to reach that goal.

    :param state_msg: The current state of the robot's State Machine and data about table if applicable
    :type state_msg: std_msgs.msg.String
    """
    global home_pose, client
    state_json = json.loads(state_msg.data)
    goal = MoveBaseGoal()
    # determine goal based on state
    if state_json['id'] == states.MOVE_TO_TABLE:
        goal.target_pose.pose = table[state_json['data']['tableID']]
    elif state_json['id'] == states.MOVE_TO_HOME:
        goal.target_pose.pose = home_pose
    else:
        return

    go_to_goal(goal, state_json, 1)


def go_to_goal(goal, state_json, attempt):
    """
    Forces the robot to go to the goal. If it fails, it'll wait (for obstacles to move...) and try again until it gets
    there
    :param goal: The goal of the navstack pathfinding
    :param state_json: The state JSON (containing current state information)
    :param attempt: The attempt number
    :type goal: MoveBaseGoal
    :type state_json: object
    :type attempt: int
    """
    client.wait_for_server()  # blocks indefinitely
    #  sets goal, send to navstack server and waits for navstack to run
    goal.target_pose.header.frame_id = "/map"
    client.send_goal_and_wait(goal)

    #  after navstack completes, get its 'state' and check if reached goal
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        if state_json['id'] == states.MOVE_TO_TABLE:
            # Ensure the correct table has been reached
            speech_engine.say("is this the table you meant?")
            yes_no_listener.callback = table_confirmed
        elif state_json['id'] == states.MOVE_TO_HOME:
            print 'successfully reached home'
            action_data = {
                'id': actions.ARRIVED,
                'data': {}
            }
            state_pub.publish(String(json.dumps(action_data)))
    else:
        # Failed to reach goal
        # leaving relocalisation test code in, as may be useful if we test to compare the two methods
        # rospy.wait_for_service('global_localization')
        # global_localization = rospy.ServiceProxy('global_localization', std_srvs.srvEmpty)
        # resp = global_localization()
        # print 'Failed to reach goal, re-localising and trying again, attempt', attempt
        # What we'll do is wait, and try again (it's usually stuck, not delocalised)
        print 'Failed to reach goal, catching my breath and trying again, attempt', attempt
        rospy.sleep(10)
        go_to_goal(goal, state_json, attempt + 1)


def table_confirmed(right_table):
    """
    Handle whether the user says the robot has arrived at the correct or incorrect table
    :param right_table: Whether the user said the current location is correct
    :type right_table: bool
    """
    action_data = {
        'id': actions.ARRIVED if right_table else actions.WRONG_TABLE,
        'data': {}
    }
    state_pub.publish(String(json.dumps(action_data)))


def goal_callback(goal):
    """
    Prints any received goal to the screen (useful for debug)
    :param goal: The goal of the navstack path finding
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
