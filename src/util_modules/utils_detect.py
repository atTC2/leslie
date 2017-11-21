import cv2
import math
import matplotlib.pyplot as plt
import numpy
from imutils.object_detection import non_max_suppression
import sys

colour_diff_threshold = 40
fov = 120.0  # camera field of view

def detect_closest_to_thief(unmodified_image, locked_colour, distro_size):
    '''
    Given an image, detect if there are people inside it.
    If there are people in the image, return the angle and the rectangle
    of the person which most closely match the thief by shirt colour.
    If there is no one in the image, then we conclude we have lost the thief.
    NOTE: If there is someone in the image despite the thief already lost
    The detector will still pick them up and assign some probability
    that person is the thief.

    :param unmodified_image: image captured
    :type image: cv2.image/np.ndarray
    :return: Original image, angle to person most matching to thief,
    if a person was found,
    rectangle bounding box of person most matching to thief
    :rtype: numpy.ndarray, float, bool, ((float, float), (float, float))
    '''

    # use hog detector
    hog = cv2.HOGDescriptor()
    # use default people detection
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    drawn_on_image = unmodified_image.copy()

    # detect people in the image with hog detector
    (rects, weights) = hog.detectMultiScale(unmodified_image, winStride=(6, 6),
                                            padding=(16, 16), scale=1.05)

    # detect_angle_to_person(image, ((x, y), (x+w, y + h)))

    angle = 0
    lost = True
    closest_rect = None
    latest_rect_global = None
    if len(rects) != 0:
        rects = numpy.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        closest_colour_diff = sys.maxint  # some max number

        # each point in `pick` is final bounding box of person
        for (xA, yA, xB, yB) in pick:
            # print (xA, yA, xB, yB)
            # take only the shirt of the person
            new_xA = (xB - xA) / 3 + xA
            new_yA = (yB - yA) / 4 + yA
            new_xB = xB - (xB - xA) / 3
            new_yB = yB - (yB - yA) / 2
            if new_xB >= 400:
                new_xB = 399
            if new_yB >= 300:
                new_yB = 299

            mode_colour = get_mode_colour(unmodified_image,
                                          new_xA, new_yA, new_xB, new_yB)
            print 'mode', mode_colour
            cv2.rectangle(drawn_on_image, (new_xA, new_yA), (new_xB, new_yB), (0, 255, 255), 2)
            cv2.rectangle(drawn_on_image, (xA, yA), (xB, yB), (0, 255, 0), 2)

            colour_diff = euclidian_colour_diff(mode_colour, locked_colour)
            print 'diff', colour_diff
            # if shirt colour is more similar that what we had
            # this person becomes the new 'closest to thief'
            if colour_diff < closest_colour_diff:
                closest_colour_diff = colour_diff
                closest_rect = ((xA, yA), (xB, yB))
                latest_rect_global = closest_rect
                angle = detect_angle_to_person(unmodified_image, ((xA, yA), (xB, yB)), distro_size)

        lost = closest_colour_diff > colour_diff_threshold

    return drawn_on_image, angle, lost, closest_rect, latest_rect_global


def euclidian_colour_diff(colour1, colour2):
    '''
    Given two colours, calculate the their difference as
    euclidian distance = sqrt(r^2+g^2+b^2)
    '''
    gdiff = colour2[0] - colour1[0]
    bdiff = colour2[1] - colour1[1]
    rdiff = colour2[2] - colour1[2]

    gdiff = gdiff * gdiff
    bdiff = bdiff * bdiff
    rdiff = rdiff * rdiff

    return math.sqrt(gdiff + bdiff + rdiff)


def make_r_g_b():
    """
    Make RGB array counter thing.
    :return: A tuple of arrays representing RGB value counts set at 0.
    :rtype: tuple of int[]
    """
    return [0 for _ in range(0, 256)], [0 for _ in range(0, 256)], [0 for _ in range(0, 256)]


def mode_it(image, r, g, b, left, up, right, down):
    """
    Adds to the RGB arrays the colours for the pixels in the image in the bounds.
    :param image: The image
    :param r: The R array
    :param g: The G array
    :param b: The B array
    :param left: The left x value to crop
    :param up: The top y value to crop
    :param right: The right x value to crop
    :param down: The bottom y value to crop
    :type image: numpy.ndarray
    :type r: int[]
    :type g: int[]
    :type b: int[]
    :type left: int
    :type up: int
    :type right: int
    :type down: int
    :return: None
    :rtype: None
    """
    for y in range(up, down):
        for x in range(left, right):
            pixel = image[y][x]
            b[pixel[0]] += 1
            g[pixel[1]] += 1
            r[pixel[2]] += 1


def get_mode(r, g, b):
    """
    Gets the mode RGB values from the RGB counts.
    :param r: The R array
    :param g: The G array
    :param b: The B array
    :type r: int[]
    :type g: int[]
    :type b: int[]
    :return: The mode RGB values
    :rtype: tuple of int
    """
    return index_of_max(r), index_of_max(g), index_of_max(b)


def get_mode_colour(image, left, up, right, down, histogram=False):
    """
    For all the pixels in the rectangle, representing a person,
    for each component r,g,b in that pixel, tally their value for
    their respective component.
    Return the mode value for each component.

    :param image: the original image
    :type image: numpy.array
    :param left: bounding box top left x position
    :type left: int
    :param up: bounding box top left y position
    :type up: int
    :param right: bounding box bottom right x position
    :type right: int
    :param down: bounding box bottom right y position
    :type down: int
    :return: The mode RGB values
    :rtype: tuple of int
    """
    r, g, b = make_r_g_b()

    mode_it(image, r, g, b, left, up, right, down)

    range_array = range(0, 256)
    if histogram:
        plt.figure(200)
        plt.clf()
        plt.cla()
        plt.plot(range_array, b, 'b', range_array, g, 'g', range_array, r, 'r')  # including h here is crucial

        plt.pause(0.0001)
    return get_mode(r, g, b)


def index_of_max(arr):
    '''
    Given an array of values,
    returns the index of the highest value
    '''
    max_val = 0
    max_index = 0
    for i in range(0, len(arr)):
        if arr[i] >= max_val:
            max_val = arr[i]
            max_index = i
    return max_index


def detect_angle_to_person(image, rectangle, distro_size):
    '''
    Given an image and points describing a rectangle,
    which represent a person,
    calculate the angle to turn to face that person.

    :param image: The image to detect from
    :param rectangle: 2 points, top left and bottom right,
    of a rectangle outlining a person in the image
    :type image: cv2.image/numpy.ndarray
    :type rectangle:((float, float),(float, float))
    :return: angle to person from centre of camera
    :rtype: float
    '''
    global fov
    top_left = rectangle[0][0]
    bottom_right = rectangle[1][0]
    if (bottom_right >= distro_size):
        bottom_right = distro_size - 1

    mid_image = image.shape[1] / 5.0

    centre = ((top_left + bottom_right) / 2)

    fixed_angle = 1

    if centre <= mid_image:
        return -fixed_angle
    if centre > mid_image and centre < (mid_image)*4:
        return 0
    if centre >= (mid_image * 4):
        return fixed_angle


def get_distance(((xA, yA), (xB, yB)), depth_history):
    '''
    Get the distance to a bounding box, represents a person

    :param: points, top-left and bottom-right of the bounding box
    :type: ((float, float), (float, float))
    :return: distance from camera to person
    :rtype: float
    '''
    saved_history = depth_history[:]
    avg_distance = 0
    for depth_image in saved_history:
        avgX = (xB + xA) / 2
        avgY = (yB + yA) / 7
        x_diff_offset = (xB - xA) * 5 / 100
        y_diff_offset = (yB - yA) * 15 / 100

        xB = avgX + x_diff_offset
        xA = avgX - x_diff_offset
        yB = 3 * avgY + y_diff_offset
        yA = 3 * avgY - y_diff_offset
        depth_image = depth_image[yA:yB, xA:xB]

        values = filter(lambda a: a != 0, depth_image.flatten())
        if (len(values) > 0):
            avg_distance = numpy.array(values).mean()

    if avg_distance == 666666:
        return 0
    return avg_distance / 1000.0


def init_distro(distro_size):
    '''
    Returns an evenly distributed probabililty distribution
    '''
    return [1.0 / distro_size for i in range(0, 400)]


def normpdf(x, mean, sd):
    '''
    Returns a value for a probability distribution
    based on the index, the mean value and the standard deviation

    :param x: index
    :type x: float
    :param mean: mean value
    :type mean: float
    :param sd: the standard deviation
    :type sd: float
    '''
    var = float(sd) ** 2
    pi = 3.1415926
    denom = (2 * pi * var) ** .5
    num = math.exp(-(float(x) - float(mean)) ** 2 / (2 * var))
    return num / denom


def update_distro(mean, std_dev, distro, distro_size):
    '''
    Updates a probability distribution by creating a new distribution
    based on the passed in values of the mean (angle as pixel index) and
    the standard deviation,
    then add that newly created distribution to the old one
    multiplied by weighting. Afterwards normalise etc.

    :param mean: value to create distribution around,
    usually middle pixel of boudning box of person
    :param type: float
    :param std_dev: standard deviation for creating distribution
    :param type: float
    :param distro:  distribution to update
    :type distro: float[]
    :param distro_size:  size of distribution
    :type distro: float
    '''
    minimum_value = 0.00001
    importance = 7

    other_distro = [normpdf(i, mean, std_dev) for i in range(0, distro_size)]

    for i in range(0, 400):
        if (other_distro[i] < minimum_value):
            other_distro[i] = minimum_value

    other_distro = [float(i) / sum(other_distro) for i in other_distro]

    for i in range(0, 400):
        distro[i] = importance * distro[i] + other_distro[i]
        if (distro[i] < minimum_value):
            distro[i] = minimum_value

    distro = [float(i) / sum(distro) for i in distro]

    range_array = range(0, 400)
    plt.figure(100)
    plt.clf()
    plt.cla()
    plt.plot(range_array, distro, 'b', range_array, other_distro, 'r')  # including h here is crucial

    plt.pause(0.0001)
    return distro.index(max(distro))
