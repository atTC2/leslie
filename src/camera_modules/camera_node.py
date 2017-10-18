import numpy as np
import cv2
import barcode_node

def get_data_from_camera(camera_checker, color):

    # 0 should be your in-build camera, 1 should be your external plugged-in camera
    cap = cv2.VideoCapture(1)

    # Force change the resolution if you want
    # cap.set(3, 1280.)
    # cap.set(4, 720.)

    data = None
    while(data == None):
        # Get the current frame
        ret, frame = cap.read()

        # Apply the method
        data = camera_checker(frame)

        # Choose whether or not you want color in your image
        # E.g. barcode scanning is done best on gray scale images
        if(color == False):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow('frame',gray)
        else:
            cv2.imshow('frame',frame)

        # Publish
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Once done, destroy all windows
    # We can move this destroy and the VideoCapture at the begnning
    # to some sort of an init method later on
    cap.release()
    cv2.destroyAllWindows()

    # Return whatever data was found through the camera_checker method
    # passed for further planning/decision making
    return data

if __name__ == "__main__":
    get_data_from_camera(barcode_node.detect, True)
