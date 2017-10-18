#!/usr/bin/env python

import cv2
import numpy as np


# Unimplemented method for identifying the barcode in a frame
def identify_barcode(frame):
    return None


def detect(image):
    """
        Barcode detection using gradients code below is from:
        https://www.pyimagesearch.com/2014/11/24/detecting-barcodes-images-python-opencv/
    """

    # Convert the image to grayscale
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Compute the Scharr gradient magnitude representation of the images
    # in both the x and y direction
    grad_x = cv2.Sobel(grey, ddepth=cv2.cv.CV_32F, dx=1, dy=0, ksize=-1)
    grad_y = cv2.Sobel(grey, ddepth=cv2.cv.CV_32F, dx=0, dy=1, ksize=-1)

    # Subtract the y-gradient from the x-gradient
    gradient = cv2.subtract(grad_x, grad_y)
    gradient = cv2.convertScaleAbs(gradient)

    # Blur and threshold the image
    blurred = cv2.blur(gradient, (9, 9))
    _, thresh = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)

    # Construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Perform a series of erosions and dilations
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)

    # Find the contours in the thresholded image
    cnts, _ = cv2.findContours(closed.copy(),
                                 cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)

    # If no contours were found, return None
    if len(cnts) == 0:
        return None

    # Otherwise, sort the contours by area and compute the rotated
    # bounding box of the largest contour
    sorted_contours = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
    rect = cv2.minAreaRect(sorted_contours)
    box = np.int0(cv2.cv.BoxPoints(rect))

    # Draw the box on the image
    cv2.drawContours(image, [box], -1, (255, 0, 255), 2)

    # Return the bounding box of the barcode
    return identify_barcode(box)

if __name__ == '__main__':
    import camera_node
    camera_node.get_data_from_camera(detect, True)
