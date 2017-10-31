import face_recognition
import os

# TODO: Put these in a configuration file
FACES_LOCATION = os.path.dirname(__file__) + "/../../resources/known_faces/"
FACES_EXTENSION = ".jpg"


def get_known_faces():
    """
    Analyze the known faces and get their encodings in order to compare
    them to encodings retrieved from the camera.
    """
    filenames = get_known_faces_filenames()
    print "filename", filenames
    saved_encodings = []

    for filename in filenames:
        # Assuming the input faces only contain the image of one individual
        # Only the first recognize face
        known_image = face_recognition.load_image_file(
                            FACES_LOCATION + filename + FACES_EXTENSION)
        saved_encodings.append(face_recognition.face_encodings(known_image)[0])

    return saved_encodings


def get_known_faces_filenames():
    """
    List all the known faces filenames in a directory and return them.
    """
    face_list = []

    for filename in os.listdir(FACES_LOCATION):
        if filename.endswith(FACES_EXTENSION):
            face_list.append(os.path.splitext(filename)[0])

    return face_list


def get_matching_name(index):
    return get_known_faces_filenames()[index]
