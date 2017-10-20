#!/usr/bin/env python


import face_recognition  # sudo pip install face_recognition
import cv2
import facial_utils

saved_encodings = facial_utils.get_known_faces()
# TODO Add these in the configuration file
frame_count = 0
frame_threshold = 30


def identify(image):
    face_locations = face_recognition.face_locations(image)
    unknown_encodings = face_recognition.face_encodings(image)

    # Draw the recognized locations on the frame
    for (top, right, bottom, left) in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)

    indeces = []

    # For each face encoding
    for unknown_encoding in unknown_encodings:
        # Compare it to the known faces
        results = face_recognition.compare_faces(saved_encodings,
                                                 unknown_encoding)

        # Identify the indeces of the matched faces
        indeces = indeces + [index for index, flag in enumerate(results)
                             if flag]

    # Get rid of any duplicates
    indeces = set(indeces)

    # Extract the original filenames using the indeces of the matched faces
    if len(indeces) > 0:
        identified = []
        for index in indeces:
            identified.append(facial_utils.get_matching_name(index))
        print "Identified: " + str(identified)

    return None


def detect(image):
    global frame_count
    data = None

    # Only pick up every other frame
    if (frame_count > frame_threshold):
        data = identify(image)
        frame_count = 0

    frame_count = frame_count + 1
    return data


if __name__ == '__main__':
    import camera_node
    camera_node.get_data_from_camera(detect, True)
