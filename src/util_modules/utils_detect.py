import cv2
import math
import matplotlib.pyplot as plt

def detect_avg_color(image, contours):
    '''
    for 1 image, and all the contours in that image
    decide what the overall average colour is throughout the contours
    '''
    avg_b = 0
    avg_g = 0
    avg_r = 0
    total = 0
    for contour in contours:
        if(len(contour) > 0):
            x, y, w, h = cv2.boundingRect(contour)
            total = w*h
            for i in range(x, x+w-1):
                for j in range(y, y+h-1):
                    avg_b += image[j][i][0]
                    avg_g += image[j][i][1]
                    avg_r += image[j][i][2]

        return (avg_b/total, avg_g/total, avg_r/total)
    return None

def detect_avg_color2(image, tl_point, br_point):
    '''
    for 1 image, and a rect in that image
    decide what the overall average colour is throughout the contours
    '''
    avg_b = 0
    avg_g = 0
    avg_r = 0
    total = 0

    if br_point[0] > 400:
        br_point[0] = 400
    if br_point[1] > 300:
        br_point[1] = 300

    x = tl_point[0]
    y = tl_point[1]
    w = br_point[0] - tl_point[0]
    h = br_point[1] - tl_point[1]
    total = w*h
    for i in range(x, x+w-1):
        for j in range(y, y+h-1):
            avg_b += image[j][i][0]
            avg_g += image[j][i][1]
            avg_r += image[j][i][2]
    #assert total != 0?

    print ('Detected', avg_b/total, avg_g/total, avg_r/total)
    return (avg_b/total, avg_g/total, avg_r/total)

def euclidian_colour_diff(colour1, colour2):
    gdiff = colour2[0] - colour1[0]
    bdiff = colour2[1] - colour1[1]
    rdiff = colour2[2] - colour1[2]

    gdiff = gdiff * gdiff
    bdiff = bdiff * bdiff
    rdiff = rdiff * rdiff

    return math.sqrt(gdiff + bdiff + rdiff)


def make_histogram(image, left, up, right, down, histogram):
    b = [0 for _ in range(0, 256)]
    g = [0 for _ in range(0, 256)]
    r = [0 for _ in range(0, 256)]
    for y in range(up, down):
        for x in range(left, right):
            pixel = image[y][x]
            b[pixel[0]] += 1
            g[pixel[1]] += 1
            r[pixel[2]] += 1

    range_array = range(0, 256)
    if histogram:
        plt.clf()
        plt.cla()
        plt.plot(range_array, b, 'b', range_array, g, 'g', range_array, r, 'r')  # including h here is crucial

        plt.pause(0.0001)
    return index_of_max(r), index_of_max(g), index_of_max(b)


def index_of_max(arr):
    max_val = 0
    max_index = 0
    for i in range(0, len(arr)):
        if arr[i] >= max_val:
            max_val = arr[i]
            max_index = i
    return max_index
