#!/usr/bin/env python

import json

import actionlib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from nav_msgs.srv import GetMap
from std_msgs.msg import String

from state_machine import states, actions
from util_modules import utils_maths

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
    global home_pose, client
    state_json = json.loads(state_msg.data)
    goal = MoveBaseGoal()
    # determine goal based on state
    if state_json['id'] == states.MOVE_TO_TABLE:
        goal.target_pose.pose = table[state_json['data']['tableID']]
    elif state_json['id'] == states.MOVE_TO_HOME:
        goal.target_pose.pose = home_pose
    elif state_json['id'] == "FOLLOW_PERP":
        follow_callback(state_msg)
        return
    else:
        return

    client.wait_for_server()  # blocks indefinitely
    #  sets goal, send to navstack server and waits for navstack to run
    goal.target_pose.header.frame_id = "/map"
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

def follow_callback(data):
    global goal_handler, current_pose
    state_json = json.loads(data.data)
    if state_json['id'] != 'FOLLOW_PERP':
        return
    print 'Good to go'

    state_data = state_json['data']
    # sends a goal to get robot to turn 90 left or right
    angle = state_data['angle']
    dist = state_data['distance']
    print 'calc goal'
    goal_pose = utils_maths.new_point(current_pose, -1 * angle, dist)
    print 'got goal'
    
    if goal_handler != None:
        goal_handler.cancel()
    goal_handler = client.send_goal(goal_pose)

def current_pose_callback(data):
    global current_pose
    print 'GETTING AMCL_POSE'
    current_pose = data.pose.pose


rospy.init_node('navstack_supervisor')
state_pub = rospy.Publisher('/action', String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, current_pose_callback, queue_size=10)

# TESTING
# state_data = {'id': states.MOVE_TO_TABLE, 'data': {'tableID': 1}}
# state_callback(String(json.dumps(state_data)))

rospy.spin()
