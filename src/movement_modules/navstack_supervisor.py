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

def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.
    
    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()
    
    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.
    
    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0
 
    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)
 
    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank) 
    # quaternion. Order is important! Original orientation is the second 
    # argument rotation which will be applied to the quaternion is the first 
    # argument. 
    return multiply_quaternions(q_headingChange, q_orig)

def create_quanternion_from_yaw(yaw):
    """
    Creates a quanternion with a given yaw

    :Args:
       | yaw (float): The yaw to set on the quanternion (in radians)
    :Return:
       | (geometry_msgs.msg.Quaternion): The generated quanternion object with given yaw
    """
    quant = Quaternion()
    quant.x = 0
    quant.y = 0
    quant.z = 0
    quant.w = 1

    return rotateQuaternion(quant, yaw)

rospy.wait_for_service('static_map')
map_getter = rospy.ServiceProxy('static_map', GetMap)
meta_data = map_getter().map.info
map_origin = meta_data.origin
map_resolution = meta_data.resolution

state_name = state_machine.States.MOVE_TO_TABLE
this_state = String(state_name)

table_orientation = Quaternion(0.0, 0.0, -0.200032655658, 0.979789230738)
table = [Pose()] * 5
for t in table:
    t.orientation = table_orientation
    print t

table[0].position = Point(-2.03, -4.8, 0)
table[1].position = Point(-1.09, -1.957, 0)
table[2].position = Point(0.193, 0.385, 0)
table[3].position = Point(1.221, 3.015, 0)
table[4].position = Point(2.282, 5.741, 0)


def callback(state):
    global this_state, table
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
    client.send_goal_and_wait(goal)
    print 'done waiting'

    print str(client.get_result())

    if client.get_result() == actionlib.GoalStatus.SUCCEEDED:        
        state_pub.publish(state_machine.Actions.ARRIVED_AT_TABLE)
    else:
        print 'fail to reach goal'
        
    # maybe a timeout for lost and go back home?

def launch_move_base():
    # launch the .launch file for move_base (navstack)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ['./launch/move_base.launch'])
    launch.start()

def goal_callback(goal):
    print str(goal)

state_pub = rospy.Publisher('/action', String, queue_size=1)
rospy.Subscriber('/state', String, callback, queue_size=1)
#rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)

if __name__ == '__main__':
    rospy.init_node('navstack_supervisor')
    callback(this_state)
    #rospy.spin()
