#!/usr/bin/env python

import json
import math
import numpy
from cv_bridge import CvBridge
from threading import Lock, Thread

import cv2
import imutils
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from imutils.object_detection import non_max_suppression
from sensor_msgs.msg import Image
from std_msgs.msg import String

from state_machine import states, state_util
from util_modules import utils_detect

history = []
history_rgb = []
max_history = 10
latest_rekt_global = None
global_lock = Lock()
distro_lock = Lock()
distro = []
distro_size = 400

colour_diff_threshold = 40

locked_colour = None
last_degree = None
fov = 120.0

waypoint_pub = rospy.Publisher('/waypoint', String, queue_size=10)
odom_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

state_id = state_util.get_start_state()


def detect_angle_to_person(image, rectangle):
    global distro_size, fov
    
    top_left = rectangle[0][0]
    bottom_right = rectangle[1][0]
    if (bottom_right >= distro_size):
        bottom_right = distro_size - 1
    
    mid_image = image.shape[1] / 2.0
    
    centre = ((top_left + bottom_right) / 2)
    
    angle = ((fov / 2.0) / mid_image) * (centre - mid_image)
    
    return angle


def detect_people(unmodified_image):
    global latest_rekt_global, locked_colour
    
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    drawn_on_image = unmodified_image.copy()
    
    # detect people in the image
    (rects, weights) = hog.detectMultiScale(unmodified_image, winStride=(6, 6),
                                            padding=(16, 16), scale=1.05)
    '''
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        padding=(8, 8), scale=1.05)
    '''
    # draw the original bounding boxes
    # for (x, y, w, h) in rects:
    #    cv2.rectangle(drawn_on_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
    # detect_angle_to_person(image, ((x, y), (x+w, y + h)))
    
    angle = 0
    lost = True
    largest_rekt = None
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    closest_rekt = None
    if len(rects) != 0:
        rects = numpy.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        
        closest_colour_diff = 99999999  # some max number
        
        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            # print (xA, yA, xB, yB)
            new_xA = (xB - xA) / 3 + xA
            new_yA = (yB - yA) / 4 + yA
            new_xB = xB - (xB - xA) / 3
            new_yB = yB - (yB - yA) / 2
            if new_xB >= 400:
                new_xB = 399
            if new_yB >= 300:
                new_yB = 299
            mode_colour = utils_detect.make_histogram(unmodified_image, new_xA, new_yA, new_xB, new_yB, False)
            print 'mode', mode_colour
            # avg_rect_colour = utils_detect.detect_avg_color2(drawn_on_image, [new_xA, new_yA], [new_xB, new_yB])
            cv2.rectangle(drawn_on_image, (new_xA, new_yA), (new_xB, new_yB), (0, 255, 255), 2)
            cv2.rectangle(drawn_on_image, (xA, yA), (xB, yB), (0, 255, 0), 2)
            colour_diff = utils_detect.euclidian_colour_diff(mode_colour, locked_colour)
            print 'diff', colour_diff
            if colour_diff < closest_colour_diff:
                closest_colour_diff = colour_diff
                closest_rekt = ((xA, yA), (xB, yB))
                latest_rekt_global = closest_rekt
                angle = detect_angle_to_person(unmodified_image, ((xA, yA), (xB, yB)))
        
        lost = closest_colour_diff > colour_diff_threshold
    
    return drawn_on_image, angle, lost, closest_rekt


def get_distance(((xA, yA), (xB, yB))):
    global history
    saved_history = history[:]
    min_distance = 666666
    max_distance = 0
    avg_distance = 0
    for depth_image in saved_history:
        avgX = (xB + xA) / 2
        avgY = (yB + yA) / 7
        x_diff_offset = (xB - xA) * 5 / 100
        y_diff_offset = (yB - yA) * 15 / 100
        
        xB = avgX + x_diff_offset
        xA = avgX - x_diff_offset
        yB = 3 * avgY + y_diff_offset
        yA = 3 * avgY - y_diff_offset
        depth_image = depth_image[yA:yB, xA:xB]
        
        values = filter(lambda a: a != 0, depth_image.flatten())
        if (len(values) > 0):
            avg_distance = numpy.array(values).mean()
            min_distance = min(values)
            max_distance = max(values)
    
    if avg_distance == 666666:
        return 0
    return avg_distance / 1000.0


def save_distance(img):
    global max_history, history, global_lock
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "16UC1")
    cv_image = imutils.resize(cv_image, width=min(400, cv_image.shape[1]))
    cv_image = numpy.array(cv_image, dtype=numpy.float32)
    history.append(cv_image)
    if (len(history) > max_history):
        history = history[1:]
    
    cv_image = cv_image.copy()
    cv_image = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
    
    if (latest_rekt_global is not None):
        ((xA, yA), (xB, yB)) = latest_rekt_global
        cv2.rectangle(cv_image, (xA, yA), (xB, yB), (0, 0, 255), 2)
    
    # print ('DEPTH:', cv_image.shape)
    with global_lock:
        cv2.imshow('depthimg', cv_image)
        cv2.waitKey(1)


def rgb_color(img):
    global history_rgb, max_history, global_lock
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(img, "passthrough")
    img = imutils.resize(img, width=min(400, img.shape[1]))
    history_rgb.append(img)
    if (len(history_rgb) > max_history):
        history_rgb = history_rgb[1:]


def decide_on_thief_status():
    counter = 0
    timeout = 0
    
    while state_id == states.ALARM:
        global history_rgb, max_history, latest_rekt_global, locked_colour
        if (len(history_rgb) >= max_history):
            img = history_rgb[len(history_rgb) - 1]
            
            # Apply the method
            drawn_on_image, angle, lost, largest_rekt = detect_people(img)
            
            if largest_rekt is not None:
                ((xA, yA), (xB, yB)) = largest_rekt
                update_distro((xB - xA) / 2 + xA, 10)
                cv2.rectangle(drawn_on_image, (xA, yA), (xB, yB), (0, 0, 255), 2)
            # Make frame
            if lost:
                counter += 1
            else:
                counter = 0
                with distro_lock:
                    where_do_i_think = distro.index(max(distro))
                
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


def angle_to_prob(angle):
    global distro_size, distro
    # print msg.twist.twist.linear.x
    
    notch = distro_size / fov
    
    if (angle == 0):
        return max_in_range(int(distro_size / 2), 20, distro, distro_size)
    if (angle > 0):
        return max_in_range(int(distro_size / 2 - angle * notch), 20, distro, distro_size)
    if (angle < 0):
        return max_in_range(int(distro_size / 2 + angle * notch), 20, distro, distro_size)


def max_in_range(mean, i, distro, distro_size):
    min_mean = mean - i
    max_mean = mean + i
    if min_mean < 0:
        min_mean = 0
    if max_mean > (distro_size - 1):
        max_mean = distro_size - 1
    
    max_value = -1
    
    for j in range(min_mean, max_mean):
        if (max_value < distro[j]):
            max_value = distro[j]
    
    # print 'max', max_value, 'index', i, 'mean', mean
    return max_value


def init_distro():
    with distro_lock:
        global distro, distro_size
        distro = [1.0 / distro_size for i in range(0, 400)]
        # distro = [0.0001 for i in range(0,400)]
        # distro[0] = 0.99999


def normpdf(x, mean, sd):
    var = float(sd) ** 2
    pi = 3.1415926
    denom = (2 * pi * var) ** .5
    num = math.exp(-(float(x) - float(mean)) ** 2 / (2 * var))
    return num / denom


def update_distro(mean, std_dev):
    with distro_lock:
        global distro, distro_size
        minimum_value = 0.0000001
        importance = 3
        
        other_distro = [normpdf(i, mean, std_dev) for i in range(0, distro_size)]
        
        for i in range(0, 400):
            if (other_distro[i] < minimum_value):
                other_distro[i] = minimum_value
        
        other_distro = [float(i) / sum(other_distro) for i in other_distro]
        
        for i in range(0, 400):
            distro[i] = importance * distro[i] + other_distro[i]
            if (distro[i] < minimum_value):
                distro[i] = minimum_value
        
        distro = [float(i) / sum(distro) for i in distro]
        
        range_array = range(0, 400)
        plt.clf()
        plt.cla()
        plt.plot(range_array, distro, 'b', range_array, other_distro, 'r')  # including h here is crucial
        
        plt.pause(0.0001)
        return distro.index(max(distro))


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
    global locked_colour, waypoint_pub
    print 'LOOKING FOR THIEF'
    which_way = state['data']['which_way']
    
    # Turn
    if which_way or which_way == 'True':
        print 'TO THE RIGHT'
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': 90, 'distance': 0}}))
    if not which_way or which_way == 'False':
        print 'TO THE LEFT'
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': -90, 'distance': 0}}))
    
    locked_colour = state['data']['colour']
    init_distro()
    decide_on_thief_status()
    
    if state_id != states.ALARM:
        return
    
    # Turn back again
    if which_way or which_way == 'True':
        print 'TO THE RIGHT'
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': -90, 'distance': 0}}))
    if not which_way or which_way == 'False':
        print 'TO THE LEFT'
        waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data': {'angle': 90, 'distance': 0}}))


def shift_arr(arr, by, default):
    if by < 0:
        arr.reverse()
        res = shift_arr(arr, -by, default)
        res.reverse()
        return res
    new_arr = [default for _ in range(0, by)]
    new_arr.extend(arr[0:distro_size - by])
    return new_arr


rospy.init_node('follow_node')
# ROS node stuff
rospy.Subscriber('/state', String, callback, queue_size=1)
pub = rospy.Publisher('/action', String, queue_size=1)
rospy.Subscriber('/camera/depth/image_raw', Image, save_distance)
rospy.Subscriber('/image_view/output', Image, rgb_color, queue_size=1)
# print callback(String(json.dumps({'id': states.ALARM, 'data': {'which_way': 'True', 'colour': [108, 134, 127]}})))
rospy.spin()
