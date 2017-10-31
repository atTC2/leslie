#!/usr/bin/env python

import rospy
import roslaunch
import json
from std_msgs.msg import String
from state_machine import state_machine
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.srv import GetMap
import actionlib

rospy.wait_for_service('static_map')
map_getter = rospy.ServiceProxy('static_map', GetMap)
meta_data = map_getter().map.info
map_origin = meta_data.origin
map_resolution = meta_data.resolution


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

table_name = {'table_1': table[0],
              'table_2': table[1],
              'table_3': table[2],
              'table_4': table[3],
              'table_5': table[4]}


def callback(state_msg):
    """
    Runs if the current state is to MOVE_TO_TABLE.
    Launches the move_base launch file, which includes the navstack and amcl.
    sends the navstack a goal
    which was included in the message published with the /state topic,
    then waits for robot to reach that goal.

    :param state: The current state of the robot's State Machine
    :type state: std_msgs.msg.String
    """
    state_json = json.loads(state_msg.data)
    if state_json['id'] != state_machine.StateIDs.MOVE_TO_TABLE:
        return

    launch_move_base()

    #  create action client, interface with navstack via client-server model
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()  # blocks indefinitely

    #  creates goal, send to navstack server and waits for navstack to run
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.pose = table_name[state_json['data']]
    print str(goal.target_pose.pose)
    client.send_goal_and_wait(goal)
    print 'done waiting'

    #  after navstack completes, get its 'state' and check if reached goal
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print 'successfully reached goal'
        action_data = {'id': state_machine.ActionIDs.ARRIVED, 'data': ''}
        state_pub.publish(String(json.dumps(action_data)))
    else:
        print 'fail to reach goal'


def launch_move_base():
    """
    Launches the move_base launch file
    """

    # launch the .launch file for move_base (navstack)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # path need to be set correctly depending on machine
    launch = roslaunch.parent.ROSLaunchParent(uuid, ['./launch/move_base.launch'])
    launch.start()


def goal_callback(goal):
    """
    Prints any received goal to the screen.
    Can be quite useful for debugging.

    :param goal: The goal of the navstack pathfinding
    :type goal: PoseStamped
    """
    print str(goal)


state_pub = rospy.Publisher('/action', String, queue_size=1)
rospy.Subscriber('/state', String, callback, queue_size=1)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('navstack_supervisor')
    state_data = {'id': state_machine.StateIDs.MOVE_TO_TABLE, 'data': 'table_1'}
    callback(String(json.dumps(state_data)))
