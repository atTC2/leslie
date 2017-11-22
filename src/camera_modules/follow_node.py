#!/usr/bin/env python

import json
import numpy
import math
import time
from cv_bridge import CvBridge  # pylint: disable=F0401
from threading import Lock, Thread
import matplotlib.pyplot as plt

import cv2
import imutils
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from state_machine import states, state_util
from util_modules import utils_maths, utils_detect
from util_modules.utils_detect import detect_closest_to_thief, init_distro, detect_angle_to_person, normpdf

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

last_degree = None  # the angle calculated by action model

state_id = state_util.get_start_state()

chase = True
figure_counter = 1

def save_distance(img):
    """
    Saves the depth image to history.
    If the history is full, pop the oldest image off.

    :param img: depth image
    :type img: sensor_msgs.msg.Image
    """
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
    if len(depth_history) > max_history:
        depth_history = depth_history[1:]

    # create a colour image out of the depth image to show
    cv_image = cv_image.copy()
    cv_image = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

    # draw the bounding box around the thief if available
    if latest_rect_global is not None:
        ((xA, yA), (xB, yB)) = latest_rect_global
        cv2.rectangle(cv_image, (xA, yA), (xB, yB), (0, 0, 255), 2)

    # print ('DEPTH:', cv_image.shape)
    with global_lock:
        cv2.imshow('depthimg', cv_image)
        cv2.waitKey(1)


def rgb_color(img):
    """
    Convert the Depth image into an RGB image.
    RGB image is used for shirt colour and define angle to person

    :param img: image to convert
    :type img: nump.array/cv2.image
    """
    global history_rgb, max_history, global_lock
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(img, "passthrough")
    img = imutils.resize(img, width=min(400, img.shape[1]))
    history_rgb.append(img)
    if len(history_rgb) > max_history:
        history_rgb = history_rgb[1:]


def decide_on_thief_status():
    """
    Detects where the thief is from camera input.
    From the detection information, update the probability distribution
    that reflects the 'belief' of potential angle and distance
    the thief is actually away from the robot.
    This code is just for showing who system believes to be thief.
    After a timeout the system stops trying to detect the thief,
    this also occurs if the system has lost the thief for > 200 frames.
    """
    global locked_colour
    global history_rgb, max_history, latest_rect_global, locked_colour
    global distro, distro_size, figure_counter
    counter_time = time.time()
    start_time = time.time()

    while state_id == states.ALARM:
        if len(history_rgb) >= max_history:
            img = history_rgb[len(history_rgb) - 1]

            # Apply the method
            drawn_on_image, angle, lost, closest_rect, latest_rect = detect_closest_to_thief(img, locked_colour, distro_size, figure_counter)
            latest_rect_global = latest_rect
            if closest_rect is not None:
                ((xA, yA), (xB, yB)) = closest_rect
                with distro_lock:
                    update_distro((xB - xA) / 2 + xA, 10)
                cv2.rectangle(drawn_on_image, (xA, yA), (xB, yB), (0, 0, 255), 2)
            # Make frame
            if not lost:
                counter_time = time.time()
                with distro_lock:
                    where_do_i_think = distro.index(max(distro))

                # Following code is for following people which is not completed
                if chase:
                    angle = detect_angle_to_person(drawn_on_image, ((where_do_i_think - 1, 0), (where_do_i_think + 1, 300)), distro_size)
                    waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': angle, 'distance': 0.5}}))

                cv2.rectangle(drawn_on_image, (where_do_i_think - 1, 0), (where_do_i_think + 1, 300), (255, 0, 0), 2)
            with global_lock:
                cv2.imshow('actualimage', drawn_on_image)
                cv2.waitKey(1)

            # print 'time1', time.time() - start_time
            if time.time() - start_time > 50:
                print 'Timed out'
                return

            # print 'time2', time.time() - counter_time
            if time.time() - counter_time > 22:
                print 'Counter limit'
                return


def update_distro(mean, std_dev):
    """
    Updates a probability distribution by creating a new distribution
    based on the passed in values of the mean (angle as pixel index) and
    the standard deviation,
    then add that newly created distribution to the old one
    multiplied by weighting. Afterwards normalise etc.

    :param mean: value to create distribution around,
    usually middle pixel of boudning box of person
    :type mean: float
    :param std_dev: standard deviation for creating distribution
    :type std_dev: float
    """
    global distro, distro_size, figure_counter
    minimum_value = 0.00001
    importance = 7

    other_distro = [normpdf(i, mean, std_dev) for i in range(0, distro_size)]

    for i in range(0, 400):
        if other_distro[i] < minimum_value:
            other_distro[i] = minimum_value

    other_distro = [float(i) / sum(other_distro) for i in other_distro]

    for i in range(0, 400):
        distro[i] = importance * distro[i] + other_distro[i]
        if distro[i] < minimum_value:
            distro[i] = minimum_value

    distro = [float(i) / sum(distro) for i in distro]

    range_array = range(0, 400)
    plt.figure(figure_counter)
    plt.clf()
    plt.cla()
    plt.plot(range_array, distro, 'b', range_array, other_distro, 'r')

    plt.pause(0.0001)
    return distro.index(max(distro))


def read_odom(msg):
    global last_degree, distro, distro_size
    if len(distro) > 0:
        with distro_lock:
            minimum_value = min(distro)
            quant = Quaternion()
            quant.x = 0
            quant.y = 0
            quant.z = msg.pose.pose.orientation.z
            quant.w = msg.pose.pose.orientation.w

            if last_degree is None:
                last_degree = math.degrees(utils_maths.yawFromQuaternion(quant))
            else:
                degree_new = math.degrees(utils_maths.yawFromQuaternion(quant))
                if degree_new != last_degree:
                    degree_change = degree_new - last_degree
                    last_degree = degree_new
                    notch = distro_size / utils_detect.fov
                    overall_change = degree_change * notch

                    distro = shift_arr(distro, int(overall_change), minimum_value)
                    distro = [float(i) / sum(distro) for i in distro]


def shift_arr(arr, by, default):
    if by < 0:
        arr.reverse()
        res = shift_arr(arr, -by, default)
        res.reverse()
        return res
    new_arr = [default for _ in range(0, by)]
    new_arr.extend(arr[0:distro_size - by])
    return new_arr


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
    """
    When the system alarms,
    turn left or right depending on which way the thief was
    detected to have came from, and start recoding video.
    After a certain timeout, turn back to the table its at,
    unless of course the owner has returned.

    :param state: The full state data
    :type state: dict
    """
    global locked_colour, waypoint_pub, distro, distro_size, figure_counter
    print 'LOOKING FOR THIEF'
    which_way = state['data']['which_way']

    # Turn
    if which_way:
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': 90, 'distance': 0}}))
    else:
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': -90, 'distance': 0}}))

    locked_colour = state['data']['colour']
    with distro_lock:
        distro = init_distro(distro_size)
    decide_on_thief_status()

    plt.close(figure_counter)
    plt.close(figure_counter + 1)

    figure_counter += 2

    if state_id != states.ALARM:
        return

    backhome_pub.publish(json.dumps({'id': state}))


rospy.init_node('follow_node')
# ROS node stuff
rospy.Subscriber('/state', String, callback, queue_size=10)
pub = rospy.Publisher('/action', String, queue_size=10)
backhome_pub = rospy.Publisher('/backhome', String, queue_size=1)
waypoint_pub = rospy.Publisher('/waypoint', String, queue_size=1)
rospy.Subscriber('/camera/depth/image_raw', Image, save_distance, queue_size=1)
rospy.Subscriber('/image_view/output', Image, rgb_color, queue_size=1)
rospy.Subscriber('/odom', Odometry, read_odom, queue_size=1)
state_util.prime_state_callback_with_starting_state(callback)
rospy.spin()
