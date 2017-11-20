import json

import rospy
from std_msgs.msg import String

import cv2

from state_machine import states


state_id = states.AT_HOME


def _state_callback(state_msg):
    """
    Method called every `/state` change.
    Updates the global variables `state_id` and `state_data`.
    :param state_msg: The state change message.
    :type state_msg: String
    """
    global state_id
    state_id = json.loads(state_msg.data)['id']


rospy.Subscriber('/state', String, _state_callback, queue_size=10)


def get_data_from_camera(camera_index, camera_checker, *accepted_states):
    """
    Gets a camera and processes the frames with the function given
    :param camera_index: The camera to get frames from
    :param camera_checker: Image processing function
    :param accepted_states: The states that the loop is allowed to run in
    :type camera_index: int
    :type camera_checker: (numpy.ndarray) -> T
    :type accepted_states: *str
    :return: data (currently always None)
    :rtype: T or None
    """
    global state_id
    cap = cv2.VideoCapture(camera_index)

    data = None
    while data is None and state_id in accepted_states:
        # Get the current frame
        ret, frame = cap.read()

        # Apply the method
        data = camera_checker(frame)

        # Make frame
        cv2.imshow('Face View', frame)

        # Publish frame
        cv2.waitKey(1)

    # Once done, destroy all windows
    # We can move this destroy and the VideoCapture at the beginning
    # to some sort of an init method later on
    cap.release()
    cv2.destroyAllWindows()

    # Return whatever data was found through the camera_checker method
    # passed for further planning/decision making
    return data
