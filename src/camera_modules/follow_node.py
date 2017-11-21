#!/usr/bin/env python

import json
import numpy
from cv_bridge import CvBridge
from threading import Lock, Thread

import cv2
import imutils
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

from state_machine import states, state_util
from util_modules.utils_detect import detect_closest_to_thief, init_distro, update_distro

# --- array variables holding data on images current and previous ---
depth_history = []  # history of depth images
history_rgb = []  # history of rgb images (same raw image as depth)
max_history = 10  # max history size
# -------------------------------------------

latest_rect_global = None  # most up to date rect describing thief
global_lock = Lock()  # Thread lock for image showing
locked_colour = None  # colour of thief's shirt

# --------------------------------------------------
# -- we represent the system's `belief` of the location of the thief
# -- as a probability distribution over the possible angles

distro_lock = Lock()  # Thread lock for updating 'belief' distribution
distro = []  # distribution representing `belief`
distro_size = 400  # size of distribution
# --------------------------------------------------

last_degree = None  # the angle calculated by actino model
fov = 120.0  # camera field of view

waypoint_pub = rospy.Publisher('/waypoint', String, queue_size=10)
odom_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

state_id = state_util.get_start_state()


def save_distance(img):
    '''
    Saves the depth image to history.
    If the history is full, pop the oldest image off.

    :param img: depth image
    :type img: sensor_msgs.msg.Image
    '''
    global max_history, depth_history, global_lock

    # use a cv bridge to convert depth image into c2 image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "16UC1")
    # resize the image width to be 400px or less, don't need full fov
    cv_image = imutils.resize(cv_image, width=min(400, cv_image.shape[1]))
    cv_image = numpy.array(cv_image, dtype=numpy.float32)
    # add to history
    depth_history.append(cv_image)
    # pop oldest if reach max size of history
    if(len(depth_history) > max_history):
        depth_history = depth_history[1:]

    # create a colour image out of the depth image to show
    cv_image = cv_image.copy()
    cv_image = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

    # draw the bounding box around the thief if available
    if (latest_rect_global is not None):
        ((xA, yA), (xB, yB)) = latest_rect_global
        cv2.rectangle(cv_image, (xA, yA), (xB, yB), (0, 0, 255), 2)

    # print ('DEPTH:', cv_image.shape)
    with global_lock:
        cv2.imshow('depthimg', cv_image)
        cv2.waitKey(1)


def rgb_color(img):
    '''
    Convert the Depth image into an RGB image.
    RGB image is used for shirt colour and define angle to person

    :param img: image to convert
    :type img: nump.array/cv2.image
    '''
    global history_rgb, max_history, global_lock
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(img, "passthrough")
    img = imutils.resize(img, width=min(400, img.shape[1]))
    history_rgb.append(img)
    if (len(history_rgb) > max_history):
        history_rgb = history_rgb[1:]


def decide_on_thief_status():
    '''
    Detects where the thief is from camera input.
    From the detection information, update the probability distribution
    that reflects the 'belief' of potential angle and distance
    the thief is actually away from the robot.
    This code is just for showing who system believes to be thief.
    After a timeout the system stops trying to detect the thief,
    this also occurs if the system has lost the thief for > 200 frames.

    '''
    global locked_colour
    global history_rgb, max_history, latest_rect_global, locked_colour
    global distro, distro_size
    counter = 0
    timeout = 0

    while state_id == states.ALARM:
        if (len(history_rgb) >= max_history):
            img = history_rgb[len(history_rgb) - 1]

            # Apply the method
            drawn_on_image, angle, lost, closest_rect, latest_rect = detect_closest_to_thief(img, locked_colour)
            latest_rect_global = latest_rect
            if closest_rect is not None:
                ((xA, yA), (xB, yB)) = closest_rect
                with distro_lock:
                    update_distro((xB - xA) / 2 + xA, 10, distro, distro_size)
                cv2.rectangle(drawn_on_image, (xA, yA), (xB, yB), (0, 0, 255), 2)
            # Make frame
            if lost:
                counter += 1
            else:
                counter = 0
                with distro_lock:
                    where_do_i_think = distro.index(max(distro))

                # Following code is for following people which is not completed
                # angle = detect_angle_to_person(drawn_on_image, ((where_do_i_think - 1, 0), (where_do_i_think + 1, 300)))
                # waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': angle, 'distance': 0.5}}))
                
                cv2.rectangle(drawn_on_image, (where_do_i_think - 1, 0), (where_do_i_think + 1, 300), (255, 0, 0), 2)
            with global_lock:
                cv2.imshow('actualimage', drawn_on_image)
                cv2.waitKey(1)

            timeout += 1

            if timeout == 500:
                print 'Timed out'
                return

            if counter == 200:
                print 'Counter limit'
                return


def callback(state_msg):
    """
    Handles new state information, and watches for faces if the state is right
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global state_id

    state = json.loads(state_msg.data)
    state_id = state['id']
    if state['id'] == states.ALARM:
        Thread(target=lookout_for_thief, args=[state]).start()


def lookout_for_thief(state):
    '''
    When the system alarms,
    turn left or right depending on which way the thief was
    detected to have came from, and start recoding video.
    After a certain timeout, turn back to the table its at,
    unless of course the owner has returned.

    :param state: The full state data
    :type state: dict (json)
    '''
    global locked_colour, waypoint_pub, distro, distro_size
    print 'LOOKING FOR THIEF'
    which_way = state['data']['which_way']

    # Turn
    if which_way == 'True':
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': 90, 'distance': 0}}))
    if which_way == 'False':
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': -90, 'distance': 0}}))

    locked_colour = state['data']['colour']
    with distro_lock:
        distro = init_distro(distro_size)
    decide_on_thief_status()

    if state_id != states.ALARM:
        return

    backhome_pub.publish(json.dumps({'id': state}))


rospy.init_node('follow_node')
# ROS node stuff
rospy.Subscriber('/state', String, callback, queue_size=1)
pub = rospy.Publisher('/action', String, queue_size=1)
backhome_pub = rospy.Publisher('/backhome', String, queue_size=1)
rospy.Subscriber('/camera/depth/image_raw', Image, save_distance)
rospy.Subscriber('/image_view/output', Image, rgb_color, queue_size=1)
# print callback(String(json.dumps({'id': states.ALARM, 'data': {'which_way': 'False', 'colour': [68, 88, 186]}})))
rospy.spin()