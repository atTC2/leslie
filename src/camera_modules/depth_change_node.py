#!/usr/bin/env python


import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import struct
import scipy.ndimage as ndimage
from state_machine import states, actions
from std_msgs.msg import String
import json


alarm = False
ith_frame = 0
threshold_frame = 2
last_images = []
mins = []
maxes = []
alarm_count = 0
alarm_count_threshold = 2
locking_frame_count = 20
min_pixel_change_count = 350
offset = 0.1

min_height_cropped = 200
max_height_cropped = 480
min_width_cropped = 100
max_width_cropped = 620

running = False
state_id = None
state_data = None

thief_went_right = True #True for right, False for left

def get_distance(img):
    global running, state_id, state_data, thief_went_right
    global alarm_count, ith_frame, last_images, mins, maxes, alarm

    if running == True:
        global last_image, alarm, ith_frame, threshold_frame, mins, maxes, alarm_count, alarm_count_threshold

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, "16UC1")

        cv_image = np.array(cv_image, dtype=np.float32)
        unnorm_cv_image = cv_image[min_height_cropped:max_height_cropped, min_width_cropped:max_width_cropped, 0:1]

        unnorm_cv_image = ndimage.gaussian_filter(unnorm_cv_image, sigma=(5, 5, 0), order=0)

        unnorm_cv_image = cv2.normalize(unnorm_cv_image, unnorm_cv_image, 0, 1, cv2.NORM_MINMAX)
        saved_full_image = unnorm_cv_image.copy()
        saved_full_image = cv2.cvtColor(saved_full_image, cv2.COLOR_GRAY2RGB)

        if ith_frame == threshold_frame:
            cropped = unnorm_cv_image

            if len(last_images) >= locking_frame_count:
                    count_left = 0
                    count_right = 0
                    for i in range (0, max_height_cropped - min_height_cropped):
                         for j in range (0, max_width_cropped - min_width_cropped):
                             if(cropped[i][j][0] > maxes[i][j][0] or
                                cropped[i][j][0] < mins[i][j][0]):
                                if j < (max_width_cropped - min_width_cropped)/2:
                                    count_left += 1
                                else:
                                    count_right +=1
                                cv2.rectangle(saved_full_image, (j, i), (j, i), (0, 0, 255), 2)
                    if(count_left + count_right > min_pixel_change_count):
                        print "Pixel change count left / right: ", count_left, " / ", count_right
                        alarm_count += 1
                    else:
                        alarm_count = 0
                    if(count_left < count_right):
                        thief_went_right = True
                    else:
                        thief_went_right = False
            else:
                last_images.append(cropped)
                if(len(last_images) >= locking_frame_count):
                    first = True
                    for last_image in last_images:
                        if first:
                            mins = last_image.copy()
                            maxes = last_image.copy()
                            first = False
                        else:
                            for i in range (0, max_height_cropped - min_height_cropped):
                                for j in range (0, max_width_cropped - min_width_cropped):
                                    if(mins[i][j][0] > last_image[i][j][0]):
                                       mins[i][j][0] = last_image[i][j][0] - offset
                                    if(maxes[i][j][0] < last_image[i][j][0]):
                                       maxes[i][j][0] = last_image[i][j][0] + offset
                    print ("Locked")
                    pub.publish(json.dumps({'id': actions.READY_TO_LOCK, 'data': state_data}))

            if alarm_count > alarm_count_threshold:
                print "ALARM!"
                state_data['which_way'] = thief_went_right
                pub.publish(json.dumps({'id': actions.MOVEMENT_DETECTED, 'data': state_data}))
                running = False

            ith_frame = 0
            cv2.imshow('img', saved_full_image)
            cv2.waitKey(1)
        else:
           ith_frame += 1

def callback(state_msg):
    """
    Processes updates to state information
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global running
    global state_id
    global state_data
    global last_images
    global mins
    global maxes

    state_json = json.loads(state_msg.data)
    state_id = state_json['id']
    state_data = state_json['data']

    if state_id == states.LOCKING:
        running = True
    elif state_id == states.LOCKED_AND_WAITING or state_id == states.ALARM:
        # Do nothing (continue running)
        pass
    else:
        # Stop running and reset some variables
        running = False
        last_images = []
        mins = []
        maxes = []


# ROS node stuff
rospy.init_node('change_node')
rospy.Subscriber('/state', String, callback, queue_size=10)
pub = rospy.Publisher('/action', String, queue_size=10)
rospy.Subscriber('/camera/depth/image_raw', Image, get_distance)

rospy.spin()
