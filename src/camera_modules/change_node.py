#!/usr/bin/env python

import json
import cv2
import os
import subprocess
import rospy
from threading import Thread
from Queue import Queue
from datetime import datetime
from std_msgs.msg import String
from state_machine import states, actions
from camera_modules import change_util
from camera_modules.change_util import crop_to_table, convert_to_grey, calculate_contours, save_frame, draw_changes
from util_modules import config_access
from interaction_modules.reporting_modules import email_report

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)

# Frame information
FRAME_QUEUE_SIZE = config_access.get_config(config_access.KEY_FRAME_QUEUE_SIZE)
previous_frames = Queue(maxsize=FRAME_QUEUE_SIZE)

# State information
running = False
state_id = None
state_data = None


def detect_change(original_image):
    """
    Detects change between the current image and the last.
    Some code inspired by https://gist.github.com/rrama/0bd1c29c8a1c1597b1eaf63847cecbf2
    :param original_image: The image to compare to the previous (the method will return False if it is the first call)
    :type original_image: numpy.ndarray
    :return: True is change was detected, False if not
    :rtype: bool
    """
    global previous_frames
    global state_data
    global pub

    image = crop_to_table(original_image)

    grey = convert_to_grey(image)

    # If the queue isn't full, add this frame to the queue and say 'no changes'
    if not previous_frames.full():
        previous_frames.put(grey)
        return False
    elif state_id == states.LOCKING:
        # Now locked, say so
        pub.publish(json.dumps({'id': actions.READY_TO_LOCK, 'data': state_data}))

    contours = calculate_contours(grey, previous_frames.get())

    # Has there been a change?
    changed = len(contours) != 0

    # Draw the changes
    draw_changes(image, contours)

    # Save the most recent frame
    previous_frames.put(grey)

    # Add timestamp
    vertical, _ = original_image.shape[:2]
    cv2.putText(original_image, str(datetime.utcnow()), (0, vertical), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)

    # Save the image if there was change
    if changed:
        save_frame(original_image)

    return changed


def reset(cap):
    """
    Resets all configured values to null and handles video file compress and reporting if an alarm was triggered
    :param cap: The frame capture object
    :type cap: VideoCapture
    """
    global previous_frames

    # Release capture and destroy windows
    cap.release()
    # TESTING
    # cv2.destroyAllWindows()

    change_util.crop_left = None
    change_util.crop_right = None
    previous_frames = Queue(maxsize=FRAME_QUEUE_SIZE)

    if change_util.video_writer is not None:
        change_util.video_writer.release()
        change_util.video_writer = None

    if change_util.video_file is not None:
        # Compress the video
        dev_null = open(os.devnull, 'w')
        compress_result = subprocess.call(
            ['HandBrakeCLI', '-i', change_util.video_file, '-o', change_util.video_file.replace('.avi', '.mp4'), '-e',
             'x265', '-q', '20'], stdout=dev_null, stderr=subprocess.STDOUT)

        if compress_result == 0:
            # Remove old file
            os.remove(change_util.video_file)

            # We know the video is compressed, and not being written to, so we can send it here
            email_report.send_report_email(state_data['notify_owner'], change_util.video_file.replace('.avi', '.mp4'))
        else:
            # Compression failed, try with the other video anyway
            email_report.send_report_email(state_data['notify_owner'], change_util.video_file)

        change_util.video_file = None
        
    pub.publish(json.dumps({'id': actions.ALARM_HANDLED, 'data': {}}))


def run():
    """
    Runs the change detector code (finding table, watching table, sending alerts, clean up)
    """
    global running
    global pub
    global state_data

    cap = cv2.VideoCapture(config_access.get_config(config_access.KEY_CHANGE_CAMERA_INDEX))

    running = True
    while running:
        # Get the current frame
        ret, frame = cap.read()

        # Apply the method
        changed = detect_change(frame)

        if changed and state_id == states.LOCKED_AND_WAITING:
            # publish alarm
            pub.publish(json.dumps({'id': actions.MOVEMENT_DETECTED, 'data': state_data}))

        # TESTING (will actually simulate getting 'turn off' state)
        # cv2.imshow('frame', frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     pub.publish(json.dumps({'id': actions.FACE_RECOGNISED, 'data': state_data}))

    # No longer running, reset
    reset(cap)


def callback(state_msg):
    """
    Processes updates to state information
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global running
    global state_id
    global state_data

    state_json = json.loads(state_msg.data)
    state_id = state_json['id']
    state_data = state_json['data']

    if state_id == states.LOCKING:
        # Start finding table in a new thread...
        Thread(target=run).start()
    elif state_id == states.LOCKED_AND_WAITING or state_id == states.ALARM:
        # Do nothing (continue running)
        pass
    else:
        # Stop running (code will automatically clean up)
        running = False


# ROS node stuff
rospy.init_node('change_node')
rospy.Subscriber('/state', String, callback, queue_size=10)
pub = rospy.Publisher('/action', String, queue_size=10)

# TESTING
# callback(String(json.dumps({'id': states.LOCKING, 'data': {'current_owner': 'Seb'}})))

rospy.spin()
