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
table[0].position = Point(-1.75060284138, -4.99175453186, 0)
table[1].position = Point(-0.678676903248, -2.35280752182, 0)
table[2].position = Point(0.39291536808, 0.249539896846, 0)
table[3].position = Point(1.50023245811, 2.81623911858, 0)
table[4].position = Point(2.67847919464, 5.56012153625, 0)

#  --- --- --- --- --- ---

home_pose = Pose()
home_pose.position = Point(-1.38991832733, -9.28993225098, 0)
home_pose.orientation = Quaternion(0, 0, 0.982110753886, 0.188304187691)

launch = None

def state_callback(state_msg):
    """
    Runs if the current state is to MOVE_TO_TABLE or MOVE_TO_HOME
    Launches the move_base launch file, which includes the navstack and amcl.
    sends the navstack a goal, which is either a table or home where
    table was included in the message published with the /state topic,
    then waits for robot to reach that goal.

    :param state_msg: The current state of the robot's State Machine and data about table if applicable
    :type state_msg: std_msgs.msg.String
    """
    global launch, table_name, home_pose
    state_json = json.loads(state_msg.data)
    goal = MoveBaseGoal()
    # determine goal based on state
    if state_json['id'] == state_machine.StateIDs.MOVE_TO_TABLE:
        goal.target_pose.pose = table[state_json['data']]
    elif state_json['id'] == state_machine.StateIDs.MOVE_TO_HOME:
        goal.target_pose.pose = home_pose
    else:
        return

    launch_move_base()

    #  create action client, interface with navstack via client-server model
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()  # blocks indefinitely

    #  send goal to navstack server and waits for navstack to run    
    goal.target_pose.header.frame_id = "/map"    
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
        # TODO: Return home

    launch.shutdown()


def launch_move_base():
    """
    Launches the move_base launch file
    """
    global launch
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




if __name__ == '__main__':
    state_pub = rospy.Publisher('/action', String, queue_size=1)
    rospy.Subscriber('/state', String, state_callback, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)
    rospy.init_node('navstack_supervisor')
    state_data = {'id': state_machine.StateIDs.MOVE_TO_TABLE, 'data': 0}
    #state_data = {'id': state_machine.StateIDs.MOVE_TO_HOME, 'data': ''}
    #state_callback(String(json.dumps(state_data)))
    rospy.spin()


