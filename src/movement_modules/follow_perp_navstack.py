import json

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from navstack_supervisor import client # use same client
from state_machine import states

goal_handler = None

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)

def build_pose_to_perp(angle, distance):
    return Pose()

def follow_callback(data):
    global goal_handler
    state_json = json.loads(data.data)
    if state_json['id'] != states.FOLLOW_PERP:
        return

    state_data = state_json['data']
    current_table_pose = state_data['tableID']
    # sends a goal to get robot to turn 90 left or right
    angle = state_json['angle']
    dist = state_json['distance']
    goal_pose = build_pose_to_perp(angle, dist)

    if goal_handler != None:
        goal_handler.cancel()
    goal_handler = client.send_goal(goal_pose)    

rospy.init_node('follow_perp_navstack')
rospy.Subscriber('/state', String, follow_callback, queue_size=10)
