#!/usr/bin/env python

import json
from functools import partial

import actionlib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.srv import GetMap
from std_msgs.msg import String
# FOR TESTING: If we're trying relocalisation if we cannot reach the goal
# from std_srvs.srv import Empty

from util_modules import utils_maths
from interaction_modules.yes_no_listener import YesNoListener
from state_machine import states, actions, state_util
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
table_orientation = Quaternion(0.0, 0.0, 0.087, -0.996)
table = []
for i in range(5):
    p = Pose()
    p.orientation = table_orientation
    table.append(p)

# positions are taken from rviz
table[0].position = Point(-2, -4.8, 0)
table[1].position = Point(-1.2, -2.0, 0)
table[2].position = Point(-0.04291536808, 0.29, 0)
table[3].position = Point(1.05023245811, 2.81623911858, 0)
table[4].position = Point(2.22847919464, 5.56012153625, 0)

current_table_id = 0

#  --- --- --- --- --- ---

home_pose = Pose()
# Starting pose left of floor
home_pose.position = Point(-1.5, -9.0, 0)
home_pose.orientation = Quaternion(0, 0, 0.982110753886, 0.188304187691)
# Starting pose right of floor
# home_pose.position = Point(5.50601768494, 8.32301235199, 0)
# home_pose.orientation = Quaternion(0, 0, 0.982110753886, 0.188304187691)
# Starting pose bins in the middle
# home_pose.position = Point(-1.44410264492, 0.979160249233, 0)
# home_pose.orientation = Quaternion(0, 0, -0.206,  0.979)

#  create action client, interface with navstack via client-server model
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

yes_no_listener = YesNoListener()

goal_handler = None
current_pose = None


def state_callback(state_msg):
    """
    Runs if the current state is to MOVE_TO_TABLE or MOVE_TO_HOME.
    Sends the navstack a goal, which is either a table or home where
    table was included in the message published to the /state topic,
    then waits for robot to reach that goal.

    :param state_msg: The current state of the robot's State Machine and data about table if applicable
    :type state_msg: std_msgs.msg.String
    """
    global home_pose, client, current_table_id
    state_json = json.loads(state_msg.data)
    goal = MoveBaseGoal()
    # determine goal based on state
    if state_json['id'] == states.MOVE_TO_TABLE:
        goal.target_pose.pose = table[state_json['data']['tableID']]
        current_table_id = state_json['data']['tableID']
    elif state_json['id'] == states.MOVE_TO_HOME:
        speech_engine.say('Going home')
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
    :type state_json: dict
    :type attempt: int
    """
    client.wait_for_server()  # blocks indefinitely
    # sets goal, send to navstack server and waits for navstack to run
    goal.target_pose.header.frame_id = "/map"
    client.send_goal_and_wait(goal)

    # after navstack completes, get its 'state' and check if reached goal
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        if state_json['id'] == states.MOVE_TO_TABLE:
            # Ensure the correct table has been reached
            speech_engine.say("is this the table you meant?")
            owner = state_json['data']['current_owner']
            friend = state_json['data'].get('friend', None)
            yes_no_listener.callback = partial(table_confirmed, owner, friend)
        elif state_json['id'] == states.MOVE_TO_HOME:
            print 'successfully reached home'
            action_data = {
                'id': actions.ARRIVED,
                'data': {}
            }
            state_pub.publish(String(json.dumps(action_data)))
        elif state_json['id'] == states.ALARM:
            pass
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


def table_confirmed(owner, friend, right_table):
    """
    Handle whether the user says the robot has arrived at the correct or incorrect table
    :param owner: Owner name
    :param friend: The added friend
    :param right_table: Whether the user said the current location is correct
    :type owner: str
    :type friend: str or None
    :type right_table: bool
    """
    action_data = {
        'id': actions.ARRIVED if right_table else actions.REJECT_TABLE,
        'data': {
            'current_owner': owner,
            'friend': friend
        }
    }
    state_pub.publish(String(json.dumps(action_data)))


def goal_callback(goal):
    """
    Prints any received goal to the screen (useful for debug)
    :param goal: The goal of the navstack path finding
    :type goal: PoseStamped
    """
    print str(goal)


def follow_callback(data):
    """
    Set a new goal for the robot when following/chasing a person.
    Needs to have an ID of FOLLOW_PERP, an angle and a distance.
    :param data: The message containing the angle and distance for the new goal
    :type data: String
    """
    global goal_handler, current_pose
    state_json = json.loads(data.data)
    print "FOLLOW_CALLBACK", data
    if state_json['id'] != 'FOLLOW_PERP':
        return

    state_data = state_json['data']
    # sends a goal to get robot to turn 90 left or right
    angle = state_data['angle']
    dist = state_data['distance']
    goal_pose = utils_maths.new_point(current_pose, angle, dist)

    if goal_handler is not None:
        goal_handler.cancel()
    goal_handler = client.send_goal(goal_pose)


def current_pose_callback(data):
    """
    Return the current pose of the robot.
    :param data: The message passed from AMCL
    :type data: PoseWithCovarianceStamped
    """
    global current_pose
    current_pose = data.pose.pose  # has covariance


def go_back_to_latest_table(data):
    """
    After chasing a thief, the robot should return
    to the latest table. The current_table_id is set
    when called state_callback with the tableID parameter.
    :param data: The message passed from the publisher through the subscriber
    :type data: String
    """
    global current_table_id
    goal = MoveBaseGoal()
    goal.target_pose.pose = table[current_table_id]
    go_to_goal(goal, json.loads(data.data), 0)


# ROS node stuff
rospy.init_node('navstack_supervisor')
state_pub = rospy.Publisher('/action', String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
state_util.prime_state_callback_with_starting_state(state_callback)
rospy.Subscriber('/waypoint', String, follow_callback, queue_size=1)
rospy.Subscriber('/backhome', String, go_back_to_latest_table, queue_size=1)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback, queue_size=10)
rospy.spin()
