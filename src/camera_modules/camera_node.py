import cv2

# 0 should be your in-build camera
# 1 should be your external plugged-in camera
# TODO : Move number to config
SELECTED_VIDEO_INDEX = 0
RECORD_VIDEO = False
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
OUT = None
fourcc = cv2.cv.CV_FOURCC('M','J','P','G')

def get_data_from_camera(camera_checker, colour, automatic_record, filename):
    global OUT, FRAME_WIDTH, FRAME_HEIGHT

    cap = cv2.VideoCapture(SELECTED_VIDEO_INDEX)

    FRAME_WIDTH = int(cap.get(3))
    FRAME_HEIGHT = int(cap.get(4))

    # Force change the resolution if you want
    # cap.set(3, 1280.)
    # cap.set(4, 720.)

    if automatic_record:
        start_recording(filename)

    data = None
    while data is None:
        # Get the current frame
        ret, frame = cap.read()

        if RECORD_VIDEO:
            OUT.write(frame)

        # Apply the method
        data = camera_checker(frame)

        # Choose whether or not you want colour in your image
        # E.g. barcode scanning is done best on greyscale images
        if colour is False:
            grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow('frame', grey)
        else:
            cv2.imshow('frame', frame)

        # Publish
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if automatic_record:
        stop_recording()

    # Once done, destroy all windows
    # We can move this destroy and the VideoCapture at the beginning
    # to some sort of an init method later on
    cap.release()
    cv2.destroyAllWindows()

    # Return whatever data was found through the camera_checker method
    # passed for further planning/decision making
    return data

def start_recording(filename):
    global OUT, RECORD_VIDEO

    RECORD_VIDEO = True
    OUT = cv2.VideoWriter(filename, fourcc, 20.0 ,(FRAME_WIDTH,FRAME_HEIGHT))

def stop_recording():
    global OUT, RECORD_VIDEO

    RECORD_VIDEO = False
    OUT.release()
    OUT = None
