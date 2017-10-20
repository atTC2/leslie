import face_recognition
import os


# TODO: Put these in a configuration file
FACES_LOCATION = "../resources/known_faces/"
FACES_EXTENSION = ".jpg"


def get_known_faces():
    filenames = get_known_faces_filenames()

    known_images = []
    saved_encodings = []

    for filename in filenames:
        known_images.append(face_recognition.load_image_file(
                            FACES_LOCATION + str(filename) + FACES_EXTENSION))

    for known_image in known_images:
        # Assuming the input faces only contain the image of one individual
        # Only the first recognize face
        saved_encodings.append(face_recognition.face_encodings(known_image)[0])

    return saved_encodings


def get_known_faces_filenames():
    face_list = []

    for file in os.listdir(FACES_LOCATION):
        if file.endswith(FACES_EXTENSION):
            face_list.append(os.path.splitext(file)[0])

    return face_list


def get_matching_name(index):
    return get_known_faces_filenames()[index]
