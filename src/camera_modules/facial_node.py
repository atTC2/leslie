#!/usr/bin/env python

import face_recognition  # sudo pip install face_recognition
import cv2
import facial_utils

saved_encodings = facial_utils.get_known_faces()
# TODO Add these in the configuration file
frame_count = 0
FRAME_THRESHOLD = 30


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

        # Identify the indeces of the matched faces
        indices += [index for index, flag in enumerate(results)
                             if flag]

    # Get rid of any duplicates
    indices = set(indices)

    # Extract the original filenames using the indices of the matched faces
    if len(indices) > 0:
        identified = []
        for index in indices:
            identified.append(facial_utils.get_matching_name(index))
        print "Identified: ", identified

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


if __name__ == '__main__':
    import camera_node
    camera_node.get_data_from_camera(detect, True)
