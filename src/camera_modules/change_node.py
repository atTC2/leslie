#!/usr/bin/env python

import cv2
from Queue import Queue

# TODO serious calibration of the below values at day/evening/night
# Image pixels are BGR
TABLE_MIN_RGB_THRESHOLD = [0, 120, 220]
TABLE_MAX_RGB_THRESHOLD = [190, 255, 255]
# Minimum percentage of the image that should be retained when cropping
MIN_TABLE_WIDTH = 0.3

FRAME_QUEUE_SIZE = 30
previous_frames = Queue(maxsize=FRAME_QUEUE_SIZE)
crop_left = None
crop_right = None


def is_in_threshold_table(colour):
    """
    Checks to see if a colour is within the valid range to be counted as a table
    :param colour: The colour of the pixel to check (in BGR format)
    :type colour: list[int]
    :return: True if it is within the thresholds, False if not
    :rtype: bool
    """
    # Figure out if the colour is within the min/max threshold of being a table
    diff_b = TABLE_MIN_RGB_THRESHOLD[0] <= colour[0] <= TABLE_MAX_RGB_THRESHOLD[0]
    diff_g = TABLE_MIN_RGB_THRESHOLD[1] <= colour[1] <= TABLE_MAX_RGB_THRESHOLD[1]
    diff_r = TABLE_MIN_RGB_THRESHOLD[2] <= colour[2] <= TABLE_MAX_RGB_THRESHOLD[2]
    return diff_b and diff_g and diff_r


def iterate_over_table(image, vertical, start, stop, step):
    """
    Iterates over an image, checking if each pixel is within a given threshold. The first pixel found to be acceptable
    shall be returned, otherwise None is returned
    :param image: The image to iterate over
    :param vertical: The height of the image
    :param start: The horizontal point to begin iteration from
    :param stop: The horizontal point to stop iteration
    :param step: The direction of iteration (1 -> left to right, -1 -> right to left)
    :type image: numpy.ndarray
    :type vertical: int
    :type start: int
    :type stop: int
    :type step: int
    :return: The horizontal index of the first pixel classified as a table.
    :rtype: int or None
    """
    for wi in range(start, stop, step):
        for hi in range(0, vertical):
            if is_in_threshold_table(image[hi, wi]):
                # We've found the first point where we think we can see a table
                return wi

    return None


def crop_to_table(image):
    """
    Crops the image to only contain vertical columns including a table (or returns the whole picture if no table is
    detected). It removes space to the left of the first sign of a table and space to the right of the last sign of a
    table, crossing the image from left to right. It the crop space is too small, it shall not crop at all. The crop is
    determined by the first image that calls this method, with all following instances being cropped in the same way
    (unless reset).
    :param image: The image to crop
    :type image: numpy.ndarray
    :return: The cropped image
    :rtype: numpy.ndarray
    """
    global crop_left
    global crop_right

    # Image pixels are referenced (height index, width index)
    vertical, horizontal = image.shape[:2]
    # Have we already cropped an image?
    if crop_left is None and crop_right is None:
        print 'Original image size:', vertical, horizontal
        # Figure out where to crop (from the left)
        crop_left = iterate_over_table(image, vertical, 0, horizontal, 1)

        # Figure out where to crop (from the right)
        crop_right = iterate_over_table(image, vertical, horizontal - 1, -1, -1)

        print 'Values used to crop:', crop_left, 'to', crop_right

    # What if we failed to find the edges or they are the same or the crop is extreme?
    if crop_left == crop_right or crop_right - crop_left < horizontal * MIN_TABLE_WIDTH:
        # TODO rather than just not cropping if frame 1 fails to have table detected, leave as None and don't label
        # system as locked until we get a frame which does have a table in it
        crop_left = 0
        crop_right = horizontal

    # Crop (in x direction)
    return image[:, crop_left:crop_right]


def detect_change(image):
    """
    Detects change between the current image and the last.
    Some code inspired by https://gist.github.com/rrama/0bd1c29c8a1c1597b1eaf63847cecbf2
    TODO when integrated into main system, allow returning True/False
    :param image: The image to compare to the previous (the method will return False if it is the first call)
    :type image: numpy.ndarray
    :return: True is change was detected, False if not
    :rtype: bool
    """
    global previous_frames
    global crop_left
    global crop_right

    image = crop_to_table(image)

    # Convert the image to greyscale
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    grey = cv2.GaussianBlur(grey, (21, 21), 0)

    # If the queue isn't full, add this frame to the queue and say 'no changes'
    if not previous_frames.full():
        previous_frames.put(grey)
        # return False # TODO when implemented properly, remove comment
        return None

    # Compute the absolute difference between the current frame and first frame
    frame_delta = cv2.absdiff(previous_frames.get(), grey)
    thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]

    # Dilate the thresholded image to fill in holes, then find contours on thresholded image
    thresh = cv2.dilate(thresh, None, iterations=2)
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    # Loop over the contours
    changed = False
    for contour in contours:
        # Change found
        changed = True
        # Compute the bounding box for the contour, draw it on the frame, and update the text
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Save the most recent frame
    previous_frames.put(grey)

    # return changed # TODO when implemented properly, remove comment


if __name__ == '__main__':
    import camera_node

    camera_node.get_data_from_camera(detect_change, True)
