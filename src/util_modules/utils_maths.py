from geometry_msgs.msg import Quaternion
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

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


def new_quanternion(old_quanternion, angle):
    yaw = math.radians(angle)
    print 'received angle ', angle
    print 'yaw converted ', yaw
    return rotateQuaternion(old_quanternion, yaw)


def yawFromQuaternion(q):
    return math.atan2(2 * (q.x * q.y + q.w * q.z),
                      q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)

def convert_to_map_angle(angle):
    return angle * -1


def new_point(old_pose, cam_angle, distance):
    angle = convert_to_map_angle(cam_angle)

    print old_pose.orientation
    old_yaw =  yawFromQuaternion(old_pose.orientation)
    print 'old roration',  math.degrees(old_yaw)
    new_orientation = new_quanternion(old_pose.orientation, angle)

    print new_orientation
    new_yaw =  yawFromQuaternion(new_orientation)
    print 'after rotation', math.degrees(new_yaw)

    opposite = math.sin(new_yaw) * distance
    adjacent = math.cos(new_yaw) * distance
    print 'adjacent ', adjacent
    print 'opposite', opposite

    print 'old position ', old_pose.position.x, old_pose.position.y

    degrees = math.degrees(new_yaw)
    new_x = old_pose.position.x + adjacent
    new_y = old_pose.position.y + opposite
    '''

    '''
    print 'new position', new_x, new_y
    goal_pose = Pose()
    goal_pose.position = Point(new_x, new_y, 0)
    goal_pose.orientation = new_orientation

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.pose = goal_pose

    return goal
