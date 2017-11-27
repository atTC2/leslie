#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge  # pylint: disable=F0401
from sensor_msgs.msg import Image
import numpy as np
import scipy.ndimage as ndimage
from state_machine import states, actions, state_util
from std_msgs.msg import String
import json
from util_modules import config_access

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)

threshold_frame = config_access.get_config(config_access.KEY_THRESHOLD_FRAME)
alarm_count_threshold = config_access.get_config(config_access.KEY_ALARM_COUNT_THRESHOLD)
locking_frame_count = config_access.get_config(config_access.KEY_LOCKING_FRAME_COUNT)
min_pixel_change_count = config_access.get_config(config_access.KEY_MIN_PIXEL_CHANGE_COUNT)
offset = config_access.get_config(config_access.KEY_OFFSET)

min_height_cropped = config_access.get_config(config_access.KEY_MIN_HEIGHT_CROPPED)
max_height_cropped = config_access.get_config(config_access.KEY_MAX_HEIGHT_CROPPED)
min_width_cropped = config_access.get_config(config_access.KEY_MIN_WIDTH_CROPPED)
max_width_cropped = config_access.get_config(config_access.KEY_MAX_WIDTH_CROPPED)

ith_frame = 0
alarm_count = 0

last_images = []
mins = []
maxes = []

alarm = False
running = False
thief_went_right = True  # True for right, False for left

state_id = None
state_data = None


def get_distance(img):
    """
    This method takes in depth camera data and triggers the alarm if the
    current frame deviates too far from the first 10 frames. The check for
    deviation only runs every ith_frame to reduce processor load.
    :param img: The image from the depth camera
    :type img: raw depth camera feed
    """

    global running, state_id, state_data
    global thief_went_right, alarm_count_threshold, threshold_frame
    global last_images, mins, maxes, alarm_count, ith_frame, alarm

    # Check if we are still in the right state to keep running this loop.
    if running:

        # Convert raw depth camera feed to 16 bit greyscale cv image.
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, "16UC1")

        cv_image = np.array(cv_image, dtype=np.float32)

        cv_image = cv_image[min_height_cropped:max_height_cropped, min_width_cropped:max_width_cropped]

        cv_image = ndimage.gaussian_filter(cv_image, sigma=(5, 5), order=0)

        cv_image = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
        saved_full_image = cv_image.copy()
        saved_full_image = cv2.cvtColor(saved_full_image, cv2.COLOR_GRAY2RGB)

        if ith_frame == threshold_frame:
            if len(last_images) >= locking_frame_count:
                    count_left = 0
                    count_right = 0
                    for i in range(0, max_height_cropped - min_height_cropped):
                        for j in range(0, max_width_cropped - min_width_cropped):
                            if cv_image[i][j] > maxes[i][j] or cv_image[i][j] < mins[i][j]:
                                # Calculate what side the movement is on, for turning to follow thief.
                                if j < (max_width_cropped - min_width_cropped)/2:
                                    count_left += 1
                                else:
                                    count_right +=1
                                # Create rectangle around movement
                                cv2.rectangle(saved_full_image, (j, i), (j, i), (0, 0, 255), 2)
                    if count_left + count_right > min_pixel_change_count:
                        alarm_count += 1
                    else:
                        alarm_count = 0
                    thief_went_right = count_left < count_right
            else:
                # Collect averages for motion detection from locking_frame_count frames.
                last_images.append(cv_image)
                if len(last_images) >= locking_frame_count:
                    first_frame = True
                    for last_image in last_images:
                        # Set mins and maxes to first image.
                        if first_frame:
                            mins = last_image.copy()
                            maxes = last_image.copy()
                            first_frame = False
                        else:
                            # Find min and max values of deviations due to noise and add an offset.
                            for i in range(0, max_height_cropped - min_height_cropped):
                                for j in range(0, max_width_cropped - min_width_cropped):
                                    if mins[i][j] > last_image[i][j]:
                                        mins[i][j] = last_image[i][j] - offset
                                    if maxes[i][j] < last_image[i][j]:
                                        maxes[i][j] = last_image[i][j] + offset
                    pub.publish(json.dumps({'id': actions.READY_TO_LOCK, 'data': state_data}))

            if alarm_count > alarm_count_threshold:
                state_data['which_way'] = thief_went_right
                pub.publish(json.dumps({'id': actions.MOVEMENT_DETECTED, 'data': state_data}))
                running = False

            ith_frame = 0

            # Display cv2 image
            cv2.imshow('Table View', saved_full_image)
            cv2.waitKey(1)
        else:
            ith_frame += 1


def callback(state_msg):
    """
    Processes updates to state information
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global running, state_id, state_data, last_images, mins, maxes

    state_json = json.loads(state_msg.data)
    state_id = state_json['id']
    state_data = state_json['data']

    if state_id == states.AT_TABLE:
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
pub = rospy.Publisher('/action', String, queue_size=10)
rospy.Subscriber('/camera/depth/image_raw', Image, get_distance)
rospy.Subscriber('/state', String, callback, queue_size=10)
state_util.prime_state_callback_with_starting_state(callback)
rospy.spin()
