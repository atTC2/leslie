#!/usr/bin/env python
import rospy
import roslaunch
import math
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

state_name = state_machine.States.MOVE_TO_TABLE
this_state = String(state_name)

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


def callback(state):    
    if state is not this_state:
        return

    launch_move_base()

    # create action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server() # blocks indefinitely
    # set goal as table 1 for now
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.pose = table[1]
    print str(goal.target_pose.pose)
    client.send_goal_and_wait(goal)
    print 'done waiting'    

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print 'successfully reached goal'
        state_pub.publish(state_machine.Actions.ARRIVED_AT_TABLE)
    else:
        print 'fail to reach goal'

    # maybe a timeout for lost and go back home?

def launch_move_base():
    # launch the .launch file for move_base (navstack)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # path need to be set correctly depending on machine
    launch = roslaunch.parent.ROSLaunchParent(uuid, ['./launch/move_base.launch'])
    launch.start()

def goal_callback(goal):
    print str(goal)

state_pub = rospy.Publisher('/action', String, queue_size=1)
rospy.Subscriber('/state', String, callback, queue_size=1)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('navstack_supervisor')
    callback(this_state)
