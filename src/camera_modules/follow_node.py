#!/usr/bin/env python

from imutils.object_detection import non_max_suppression
import cv2
import camera_node
import numpy
import imutils
import json
import cv2
import copy
import scipy.stats as stats
import matplotlib.pyplot as plt
from threading import Lock
from sensor_msgs.msg import Image
#import subprocess
import rospy
import math
#from threading import Thread
#from Queue import Queue
#from datetime import datetime
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from state_machine import states, actions
from cv_bridge import CvBridge, CvBridgeError
from util_modules import utils_detect

history = []
history_rgb = []
max_history = 10
latest_rekt_global = None
global_lock = Lock()
distro_lock = Lock()
distro = []
distro_size = 400

locked_colour = None

def detect_angle_to_person(image,rectangle):

    fov = 90

    mid_image = image.shape[1]/2.0
    centre = ((rectangle[0][0] + rectangle[1][0])/2)
    if centre > mid_image:
        #print
        angle = ((fov/2.0)/mid_image) * (centre - mid_image)
    else:
        angle = ((fov/2.0)/mid_image) * (centre - mid_image)
    #print "angle: ", angle
    return angle

def detect_people(image):

    global latest_rekt_global, locked_colour

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    orig = image.copy()

    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(6, 6),
        padding=(16, 16), scale=1.05)

    '''
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        padding=(8, 8), scale=1.05)
    '''
    # draw the original bounding boxes
    #for (x, y, w, h) in rects:
    #    cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

    #detect_angle_to_person(image, ((x, y), (x+w, y + h)))

    angle = 0
    lost = True
    largest_rekt = None
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    closest_rekt = None
    if len(rects) !=0:
        rects = numpy.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        closest_colour_diff = 99999999 # some max number

        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            #print (xA, yA, xB, yB)
            cv2.rectangle(orig, (xA, yA), (xB, yB), (0, 255, 0), 2)
            avg_rect_colour = utils_detect.detect_avg_color2(orig, (xA, yA), (xB, yB))
            colour_diff = utils_detect.euclidian_colour_diff(avg_rect_colour, locked_colour)

            if colour_diff < closest_colour_diff:
                closest_colour_diff = colour_diff
                closest_rekt = ((xA, yA), (xB, yB))
                latest_rekt_global = closest_rekt
                angle = detect_angle_to_person(orig, ((xA, yA), (xB, yB)))

        #print "angle: ", angle
        lost = False

    return orig, angle, lost, closest_rekt

def get_distance(((xA,yA),(xB,yB))):
    global history
    saved_history = history[:]
    min_distance = 666666
    max_distance = 0
    avg_distance = 0
    for depth_image in saved_history:
        avgX = (xB + xA)/2
        avgY = (yB + yA)/7
	x_diff_offset = (xB - xA) * 5 / 100
	y_diff_offset = (yB - yA) * 15 / 100

	xB = avgX + x_diff_offset
	xA = avgX - x_diff_offset
	yB = 3 * avgY + y_diff_offset
	yA = 3 * avgY - y_diff_offset
        depth_image = depth_image[yA:yB, xA:xB]
        #print(xA, xB, yA, yB, depth_image.shape)
        values = filter(lambda a: a != 0, depth_image.flatten())
        if(len(values) > 0):
            avg_distance = numpy.array(values).mean()
            min_distance = min(values)
            max_distance = max(values)
    #print 'DISTANCE: ', min_distance
    #print 'DISTANCE AVG: ', avg_distance
    #print 'DISTANCE MAX: ', max_distance
    if min_distance == 666666:
        return 0
    return min_distance/1000.0

def save_distance(img):


    global max_history, history, global_lock

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "16UC1")
    cv_image = imutils.resize(cv_image, width=min(400, cv_image.shape[1]))
    cv_image = numpy.array(cv_image, dtype=numpy.float32)
    history.append(cv_image)
    if(len(history) > max_history):
        history = history[1:]

    cv_image = cv_image.copy()
    cv_image = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

    if(latest_rekt_global is not None):
        ((xA, yA), (xB,yB)) = latest_rekt_global
        cv2.rectangle(cv_image, (xA, yA), (xB,yB), (0, 0, 255), 2)

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
    if(len(history_rgb) > max_history):
        history_rgb = history_rgb[1:]


def decide_on_thief_status():
    global history_rgb, max_history, latest_rekt_global, locked_colour
    counter = 0
    close_threshold = 4000
    while True:
        if(len(history_rgb) >= max_history):
            img = history_rgb[len(history_rgb) - 1]
            data = None

            # Apply the method
            data, angle, lost, largest_rekt = detect_people(img)

            if largest_rekt is not None:
                distance = get_distance(largest_rekt)
                ((xA, yA), (xB,yB)) = largest_rekt
                where_do_i_think = update_distro((xB - xA)/2 + xA, 10)
                cv2.rectangle(img, (xA, yA), (xB,yB), (0, 0, 255), 2)
                cv2.rectangle(img, (where_do_i_think-1, 0), (where_do_i_think + 1,300), (255, 0, 0), 2)
            else:
                distance = 0
                update_distro_with_uniform()
            # Make frame
            if lost:
                counter += 1

            if counter == 600:
                if distance < close_threshold:
	            return True
	        else:
	            return False

            #if(distance != 0):
                #print 'distance: ', distance

            distance = distance - 1 #don't go inside of the person please
            # print ('ACTUAL:', img.shape)
            distance = 1
            if(angle_to_prob(angle) > 0.02):
                waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data':{'angle':angle,'distance':distance}}))

            with global_lock:
                cv2.imshow('actualimage', img)
                cv2.waitKey(1)


def angle_to_prob(angle):
    fov = 90.0
    global distro_size, distro
    #print msg.twist.twist.linear.x

    notch = distro_size / 90.0

    if(angle == 0):
        return max_in_range(int(distro_size/2), 20, distro, distro_size)
    if(angle > 0):
        return max_in_range(int(distro_size/2 + angle*notch), 20, distro, distro_size)
    if(angle < 0):
        return max_in_range(int(distro_size/2 - angle*notch), 20, distro, distro_size)


def max_in_range(mean, i, distro, distro_size):
    min_mean = mean - i
    max_mean = mean + i
    if min_mean < 0:
        min_mean = 0
    if max_mean > (distro_size - 1):
        max_mean = distro_size - 1

    max_value = -1

    for j in range(min_mean, max_mean):
        if(max_value < distro[j]):
            max_value = distro[j]

    #print 'max', max_value, 'index', i, 'mean', mean
    return max_value

def init_distro():
    with distro_lock:
        global distro, distro_size
        distro = [1.0/distro_size for i in range(0,400)]
        #distro = [0.0001 for i in range(0,400)]
        #distro[0] = 0.99999

def normpdf(x, mean, sd):
    var = float(sd)**2
    pi = 3.1415926
    denom = (2*pi*var)**.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom

def update_distro_with_uniform():
    with distro_lock:
        global distro, distro_size
        minimum_value = 0.0000001
        importance = 1000

        other_distro = [1.0/distro_size for i in range(0,400)]

        for i in range(0, 400):
            distro[i] = importance * distro[i] + other_distro[i]
            if (distro[i] < minimum_value):
                distro[i] = minimum_value

        distro = [float(i)/sum(distro) for i in distro]
        #print distro

        range_array = range(0,400)
        plt.clf()
        plt.cla()
        plt.plot( range_array, distro, 'b', range_array, other_distro, 'r') # including h here is crucial

        plt.pause(0.0001)
        return distro.index(max(distro))


def update_distro(mean, std_dev):
    with distro_lock:
        global distro, distro_size
        minimum_value = 0.0000001
        importance = 10

        other_distro = [normpdf(i, mean, std_dev) for i in range(0, distro_size)]

        for i in range(0, 400):
            if (other_distro [i] < minimum_value):
                other_distro [i] = minimum_value

        other_distro = [float(i)/sum(other_distro) for i in other_distro]

        for i in range(0, 400):
            distro[i] = importance * distro[i] + other_distro[i]
            if (distro[i] < minimum_value):
                distro[i] = minimum_value

        distro = [float(i)/sum(distro) for i in distro]
        #print distro

        range_array = range(0,400)
        plt.clf()
        plt.cla()
        plt.plot( range_array, distro, 'b', range_array, other_distro, 'r') # including h here is crucial

        plt.pause(0.0001)
        return distro.index(max(distro))

def callback(state_msg):
    """
    Handles new state information, and watches for faces if the state is right
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global locked_colour
    #global SLEEP_TIME
    init_distro()
    print "state_msg: ", state_msg
    print 'lookinf for thief?'
    state = json.loads(state_msg.data)
    action = {}
    if state['id'] == states.ALARM:
        print 'LOOKING FOR THIEF'
        which_way = state['data']['which_way']
        if which_way or which_way == 'True':
            waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data':{'angle':90,'distance':0}}))
        if not which_way or which_way == 'False':
            waypoint_pub.publish(json.dumps({'id': 'FOLLOW_PERP', 'data':{'angle':-90,'distance':0}}))
        locked_colour = state['data']['colour']
        result = decide_on_thief_status()

        print "result: ", result
        if result:
            print "caught "
            pub.publish(json.dumps({'id': actions.CAUGHT_THIEF, 'data': state['data']}))
        else:
            print "lost "
            pub.publish(json.dumps({'id': actions.LOST_THIEF, 'data': state['data']}))

        init_distro()
        result_of_chase = None


def read_odom(msg):
    with distro_lock:
        fov = 90.0
        minimum_value = 0.0000001
        global distro, distro_size
        #print msg.twist.twist.linear.x
        degree_change = math.degrees(msg.twist.twist.angular.z)
        notch = distro_size / fov / 4
        overall_change = abs(degree_change) * notch

        if(degree_change > 0):
            for i in range(0, distro_size):
                if i < distro_size - overall_change:
                   distro[i] = distro[int(i + overall_change)]
                else:
                   distro[i] = minimum_value
            distro = [float(i)/sum(distro) for i in distro]
        if(degree_change < 0):
            for i in reversed(range(0, distro_size)):
                if i > overall_change:
                   distro[i] = distro[int(i - overall_change)]
                else:
                   distro[i] = minimum_value
            distro = [float(i)/sum(distro) for i in distro]


# ROS node stuff
rospy.init_node('follow_node')
rospy.Subscriber('/state', String, callback, queue_size=1)
pub = rospy.Publisher('/action', String, queue_size=1)
waypoint_pub = rospy.Publisher('/waypoint', String, queue_size=1)
rospy.Subscriber('/camera/depth/image_raw', Image, save_distance)
rospy.Subscriber('/image_view/output', Image, rgb_color, queue_size=1)
rospy.Subscriber('/odom', Odometry, read_odom, queue_size=1)
#callback(String(json.dumps({'id': states.ALARM, 'data': ''})))
rospy.spin()
