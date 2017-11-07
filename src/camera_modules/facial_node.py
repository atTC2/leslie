#!/usr/bin/env python

import face_recognition  # sudo pip install face_recognition
import cv2
import facial_utils
import rospy
from std_msgs.msg import String
import json
from state_machine import states, actions
import camera_node
from util_modules import config_access


if __name__ != '__main__':
    from sys import stderr
    # No one should import this code (stops multiple identical nodes being started)
    print >> stderr, __name__, 'should not be imported!'
    exit(1)


saved_encodings = facial_utils.get_known_faces()
CAMERA_INDEX = config_access.get_config(config_access.KEY_CAMERA_INDEX_FACE)
FRAME_THRESHOLD = config_access.get_config(config_access.KEY_FACE_FRAME_THRESHOLD)

frame_count = 0


def identify(image):
    """
    Identify faces in a given frame by associating them with
    filenames in the /known_faces/ directory.
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
    Only attempt to recognize a face in a frame every time
    the frame_count reaches the FRAME_THRESHOLD. When the threshold
    is reached, the identification process is carried out on the
    selected frame. Consequently, the frame_count is reset.
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
    print "state_msg: ", state_msg
    state = json.loads(state_msg.data)
    action = {}
    action['data'] = {}
    if state['id'] == states.AT_TABLE:
        print "got into if statement"
        result = camera_node.get_data_from_camera(CAMERA_INDEX, detect)
        print "result: ", result
        action['id'] = actions.GOT_FACE
        action['data']['current_owner'] = result[0]
        
    elif state['id'] in [states.LOCKED_AND_WAITING, states.ALARM]:
        owner = state['data']['current_owner']
        rospy.sleep(30)
        result = camera_node.get_data_from_camera(CAMERA_INDEX, detect)
        while owner not in result:
            result = camera_node.get_data_from_camera(CAMERA_INDEX, detect)
        action['id'] = actions.FACE_RECOGNISED
        action['data']['notify_owner'] = owner
        action['data']['current_owner'] = ""
    if action['data'] != {}:
        pub.publish(json.dumps(action))


rospy.init_node("facial_node")
pub = rospy.Publisher("/action", String, queue_size=1)
rospy.Subscriber('/state', String, state_callback, queue_size=1)
rospy.spin()
