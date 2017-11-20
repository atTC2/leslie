#!/usr/bin/env python
from functools import partial
from threading import Thread

import face_recognition  # sudo pip install face_recognition
import cv2
import facial_utils
import rospy
from std_msgs.msg import String
import json

from interaction_modules.yes_no_listener import YesNoListener
from state_machine import states, actions
import camera_node
from util_modules import config_access, speech_engine

if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


saved_encodings = facial_utils.get_known_faces()
CAMERA_INDEX = config_access.get_config(config_access.KEY_CAMERA_INDEX_FACE)
FRAME_THRESHOLD = config_access.get_config(config_access.KEY_FACE_FRAME_THRESHOLD)
SLEEP_TIME = config_access.get_config(config_access.KEY_MIN_LOCKED_TIME_SECONDS)

frame_count = 0

yes_no_listener = YesNoListener()


def identify(image):
    """
    Identify faces in a given frame by associating them with file names in the /known_faces/ directory.
    :param image: The image to perform facial recognition on
    :type image: numpy.ndarray
    :return: The name of the person identified, or None if no one is detected
    :rtype: str[] or None
    """
    global saved_encodings
    face_locations = face_recognition.face_locations(image)
    unknown_encodings = face_recognition.face_encodings(image)

    # Draw the recognized locations on the frame
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)

    indices = []

    # For each face encoding
    for unknown_encoding in unknown_encodings:
        # Compare it to the known faces
        results = face_recognition.compare_faces(saved_encodings,
                                                 unknown_encoding)

        # Identify the indices of the matched faces
        indices += [index for index, flag in enumerate(results) if flag]

    # Get rid of any duplicates
    indices = set(indices)

    # Extract the original filenames using the indices of the matched faces
    if len(indices) > 0:
        identified = []
        for index in indices:
            identified.append(facial_utils.get_matching_name(index))
        print "Identified: ", identified
        return identified
    else:
        return None


def detect(image):
    """
    Only attempt to recognize a face in a frame every time the frame_count reaches the FRAME_THRESHOLD. When the
    threshold is reached, the identification process is carried out on the selected frame. Consequently, the
    frame_count is reset.
    :param image: The image to perform facial recognition on
    :type image: numpy.ndarray
    :return: The name of the person identified, or None if no one is detected
    :rtype: str[] or None
    """
    global frame_count, FRAME_THRESHOLD
    data = None

    # Only pick up every other frame
    if frame_count > FRAME_THRESHOLD:
        data = identify(image)
        frame_count = 0

    frame_count += 1
    return data


def state_callback(state_msg):
    """
    Handles new state information, and watches for faces if the state is right
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    global SLEEP_TIME

    print "state_msg: ", state_msg
    state = json.loads(state_msg.data)
    if state['id'] == states.AT_HOME:
        Thread(target=get_face).start()
        return
    elif state['id'] == states.LOCKED_AND_WAITING:
        owner = state['data']['current_owner']
        friend = state['data'].get('friend', None)
        rospy.sleep(SLEEP_TIME)
        result = []
        while owner not in result and friend not in result:
            result = camera_node.get_data_from_camera(CAMERA_INDEX, detect, states.LOCKED_AND_WAITING, states.ALARM)
            if result is None:
                return

        if owner in result:
            speech_engine.say('Welcome back ' + owner)
        if friend in result:
            speech_engine.say('Hello ' + friend)

        action = {
            'id': actions.FACE_RECOGNISED,
            'data': {
                'notify_owner': owner
            }
        }
        pub.publish(json.dumps(action))

    # If the state isn't AT_HOME, we want to make sure that it isn't waiting on some face recognition verification
    yes_no_listener.callback = None


def get_face():
    """
    Gets faces currently in frame, selects the first one, and confirms with the user whether that person is the
    intended locker
    """
    names = camera_node.get_data_from_camera(CAMERA_INDEX, detect, states.AT_HOME)
    if names is None:
        return
    print "result: ", names
    speech_engine.say('Are you ' + names[0])
    yes_no_listener.callback = partial(got_face, names[0])


def got_face(name, is_them):
    """
    Handles when a face has been detected and whether or not is the intended locker
    :param name: The name of the detected person
    :param is_them: Whether that person has been confirmed as the locker
    :type name: str
    :type is_them: bool
    """
    if not is_them:
        get_face()
        return

    speech_engine.say('hello ' + name + ', I am Leslie, the Personal Item Protection System')
    action = {
        'id': actions.FACE_DETECTED,
        'data': {
            'current_owner': name
        }
    }
    pub.publish(json.dumps(action))


rospy.init_node("facial_node")
pub = rospy.Publisher("/action", String, queue_size=10)
rospy.Subscriber('/state', String, state_callback, queue_size=10)
state_callback(String(json.dumps({"id": states.AT_HOME})))
rospy.spin()
