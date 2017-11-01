import cv2


def get_data_from_camera(camera_index, camera_checker, reset_func):
    """
    Gets a camera and processes the frames with the function given
    :param camera_index: The camera to get frames from
    :param camera_checker: Image processing function
    :param reset_func: Reset function to call when the video closes
    :type camera_index: int
    :type camera_checker: (numpy.ndarray) -> T
    :type reset_func: () -> None
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
        cv2.imshow('frame', frame)

        # Publish frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Once done, destroy all windows
    # We can move this destroy and the VideoCapture at the beginning
    # to some sort of an init method later on
    cap.release()
    cv2.destroyAllWindows()
    if reset_func is not None:
        reset_func()

    # Return whatever data was found through the camera_checker method
    # passed for further planning/decision making
    return data
