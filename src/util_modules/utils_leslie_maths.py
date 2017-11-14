from geometry_msgs.msg import Quaternion
import util


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

    return util.rotateQuaternion(quant, yaw)


def create_quanternion_from_angle(angle):
    yaw = math.radians(angles)
    return create_quanternion_from_yaw(yaw)


def new_quanternion(old_quanternion, angle, distance):
    turn_quanternion = create_quanternion_from_angle(angle)
    return turn_pose * old_quanternion
