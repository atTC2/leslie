import cv2


def get_data_from_camera(camera_index, camera_checker):
    """
    Gets a camera and processes the frames with the function given
    :param camera_index: The camera to get frames from
    :param camera_checker: Image processing function
    :type camera_index: int
    :type camera_checker: (numpy.ndarray) -> T
    :return: data (currently always None)
    :rtype: T
    """
    cap = cv2.VideoCapture(camera_index)

    data = None
    while data is None:
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
