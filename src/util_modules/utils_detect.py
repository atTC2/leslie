import cv2
import math
import matplotlib.pyplot as plt
import numpy
from imutils.object_detection import non_max_suppression
import sys
import operator

colour_diff_threshold = 45
fov = 120.0  # camera field of view

recording = False
true_colour = 'blue'
test_colour = 'grey'
arry_methody_i = 0
arry_colours_i = 0

if recording:
    file_name = 'recording.csv'
else:
    file_name = 'results.csv'

file_path = '/home/ben/Documents/experiment_colours/' + file_name
result_file = open(file_path, 'a')

data = []


def detect_closest_to_thief(unmodified_image, locked_colour, distro_size, figure_counter):
    """
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
    """
    global data, arry_methody_i, arry_colours_i, test_colour

    # use hog detector
    hog = cv2.HOGDescriptor()
    # use default people detection
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    drawn_on_image = unmodified_image.copy()

    # detect people in the image with hog detector
    (rects, weights) = hog.detectMultiScale(unmodified_image, winStride=(6, 6),
                                            padding=(16, 16), scale=1.05)

    if recording:
        test_colour = arry_colours[arry_colours_i][1]

    # detect_angle_to_person(image, ((x, y), (x+w, y + h)))
    
    angle = 0
    lost = True
    closest_rect = None
    latest_rect_global = None
    if len(rects) != 0:
        if len(rects) != 1:
            raise Exception('FUCKED UP, probably saw a plant, top shite.')
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
            
            if recording:
                got_colour = arry_methody[arry_methody_i][1](unmodified_image, new_xA, new_yA, new_xB, new_yB, False, figure_counter)
            else:
                funcy = dicty_methody[arry_colours[arry_colours_i][0]]
                print 'Running function', funcy.func_name
                got_colour = funcy(unmodified_image, new_xA, new_yA, new_xB, new_yB, False, figure_counter)
            print 'colour', got_colour
            cv2.rectangle(drawn_on_image, (new_xA, new_yA), (new_xB, new_yB), (0, 255, 255), 2)
            cv2.rectangle(drawn_on_image, (xA, yA), (xB, yB), (0, 255, 0), 2)

            if recording:
                colour_diff = euclidian_colour_diff(got_colour, locked_colour)
            else:
                print 'new locked_colour', arry_colours[arry_colours_i][2]
                colour_diff = euclidian_colour_diff(got_colour, arry_colours[arry_colours_i][2])
            print 'diff', colour_diff
            
            # TESTING
            if recording:
                data.append(got_colour)
                
                if len(data) == 10:
                    total_r = 0
                    total_g = 0
                    total_b = 0
                    for datum in data:
                        total_r += datum[0]
                        total_g += datum[1]
                        total_b += datum[2]
                    line = arry_methody[arry_methody_i][0] + ', ' + true_colour + ', (' + str(total_r / 10) + ', ' + str(total_g / 10) + ', ' + str(total_b / 10) + ')'
                    result_file.write(line + '\n')
                    result_file.flush()
                    data = []
                    arry_methody_i += 1
                    print 'WROTE DATA'
                    
                    if arry_methody_i == len(arry_methody):
                        result_file.close()
                        exit(0)
            
            else:
                data.append(colour_diff)
                if len(data) == 10:
                    data_line_section = ', '.join([str(datum) for datum in data])
                    line = arry_colours[arry_colours_i][0] + ', ' + true_colour + ', ' + test_colour + ', ' + data_line_section
                    result_file.write(line + '\n')
                    result_file.flush()
                    
                    data = []
                    arry_colours_i += 1
                    
                    if arry_colours_i == 3:
                        result_file.close()
                        exit(0)
                    
                    test_colour = arry_colours[arry_colours_i][1]
                    print 'WROTE DATA'
                    print 'new method', arry_colours[arry_colours_i][0]
                    print 'new test_colour', test_colour
            
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
    """
    Given two colours, calculate the their difference as
    euclidian distance = sqrt(r^2+g^2+b^2)
    """
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
            r[pixel[2]] += 1
            g[pixel[1]] += 1
            b[pixel[0]] += 1


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


def pixel_to_tuple(pixel):
    """
    Makes RGB
    :param pixel:
    :return:
    """
    return pixel[2], pixel[1], pixel[0]


def get_mode_colour(image, left, up, right, down, histogram, figure_counter):
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
    :param histogram: Whether you want to display the histogram
    :type histogram: bool
    :return: The mode RGB values
    :rtype: tuple of int
    """
    r, g, b = make_r_g_b()

    mode_it(image, r, g, b, left, up, right, down)

    range_array = range(0, 256)
    if histogram:
        plt.figure(figure_counter + 1)
        plt.clf()
        plt.cla()
        plt.plot(range_array, b, 'b', range_array, g, 'g', range_array, r, 'r')

        plt.pause(0.0001)
    return get_mode(r, g, b)


def get_proper_mode(image, left, up, right, down, _, _1):
    colour_counter = dict()
    for y in range(up, down):
        for x in range(left, right):
            pixel = pixel_to_tuple(image[y][x])
            colour_counter[pixel] = colour_counter.get(pixel, 0) + 1
    mode = max(colour_counter.iteritems(), key=operator.itemgetter(1))[0]
    return mode


def get_condensed_mode(image, left, up, right, down, _, _1):
    # Const
    condenser = 16
    mid_add = (256 / condenser) / 2
    
    colour_counter = dict()
    for y in range(up, down):
        for x in range(left, right):
            pixel_bgr = image[y][x]
            pixel_rgb = pixel_bgr[2] / condenser, pixel_bgr[1] / condenser, pixel_bgr[0] / condenser
            colour_counter[pixel_rgb] = colour_counter.get(pixel_rgb, 0) + 1
    mode = max(colour_counter.iteritems(), key=operator.itemgetter(1))[0]
    mode = mode[0] * condenser + mid_add, mode[1] * condenser + mid_add, mode[2] * condenser + mid_add
    return mode


def index_of_max(arr):
    """
    Given an array of values,
    returns the index of the highest value
    """
    max_val = 0
    max_index = 0
    for i in range(0, len(arr)):
        if arr[i] >= max_val:
            max_val = arr[i]
            max_index = i
    return max_index


def get_average_colour(image, left, up, right, down, _, _1):
    total_r = 0
    total_g = 0
    total_b = 0
    total_pix = 0
    for y in range(up, down):
        for x in range(left, right):
            pixel = image[y][x]
            total_r += pixel[2]
            total_g += pixel[1]
            total_b += pixel[0]
            total_pix += 1
    return total_r / total_pix, total_g / total_pix, total_b / total_pix


def detect_angle_to_person(image, rectangle, distro_size):
    """
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
    """
    global fov
    top_left = rectangle[0][0]
    bottom_right = rectangle[1][0]
    if bottom_right >= distro_size:
        bottom_right = distro_size - 1

    mid_image = image.shape[1] / 5.0

    centre = ((top_left + bottom_right) / 2)

    fixed_angle = 5

    if centre <= mid_image:
        return -fixed_angle
    if mid_image < centre < mid_image * 4:
        return 0
    if centre >= (mid_image * 4):
        return fixed_angle


def get_distance(((xA, yA), (xB, yB)), depth_history):
    """
    Get the distance to a bounding box, represents a person

    :param: points, top-left and bottom-right of the bounding box
    :type: ((float, float), (float, float))
    :return: distance from camera to person
    :rtype: float
    """
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
        if len(values) > 0:
            avg_distance = numpy.array(values).mean()

    if avg_distance == 666666:
        return 0
    return avg_distance / 1000.0


def init_distro(distro_size):
    """
    Returns an evenly distributed probabililty distribution
    """
    return [1.0 / distro_size for _ in range(0, 400)]


def normpdf(x, mean, sd):
    """
    Returns a value for a probability distribution
    based on the index, the mean value and the standard deviation

    :param x: index
    :type x: float
    :param mean: mean value
    :type mean: float
    :param sd: the standard deviation
    :type sd: float
    :return: Probability
    :rtype: float
    """
    var = float(sd) ** 2
    pi = 3.1415926
    denom = (2 * pi * var) ** .5
    num = math.exp(-(float(x) - float(mean)) ** 2 / (2 * var))
    return num / denom


# get_average_colour
# get_mode_colour
# get_proper_mode
# get_condensed_mode
dicty_methody = {
    'average': get_average_colour,
    'mode': get_mode_colour,
    'proper_mode': get_proper_mode,
    'condensed_mode': get_condensed_mode
}


arry_methody = [
    ('average', get_average_colour),
    ('mode', get_mode_colour),
    ('proper_mode', get_proper_mode),
    ('condensed_mode', get_condensed_mode)
]


# def get_colour_from(method, colour_name):
#     for colour_tuple in arry_colours:
#         if colour_tuple[0] == method and colour_tuple[1] == colour_name:
#             return colour_tuple[2]
#     raise Exception('All hell')


arry_colours = [
    
    ('average', 'grey', (32, 35, 11)), (
        'mode', 'grey', (28, 31, 11)), (
        'proper_mode', 'grey', (27, 30, 11)), (
        'condensed_mode', 'grey', (24, 24, 8)), (
        'average', 'green', (38, 41, 12)), (
        'mode', 'green', (30, 34, 10)), (
        'proper_mode', 'green', (30, 34, 9)), (
        'condensed_mode', 'green', (40, 40, 8)), (
        'average', 'blue', (45, 54, 18)), (
        'mode', 'blue', (40, 50, 18)), (
        'proper_mode', 'blue', (41, 51, 18)), (
        'condensed_mode', 'blue', (40, 56, 24)), (
        'average', 'red', (86, 44, 4)), (
        'mode', 'red', (82, 39, 0)), (
        'proper_mode', 'red', (80, 40, 3)), (
        'condensed_mode', 'red', (86, 40, 8))
]
