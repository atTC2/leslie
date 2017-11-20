import cv2
import math

def max_difference(xs):
    max_idx, max_elem = max(enumerate(xs), key=operator.itemgetter(1))

    if max_idx == 0:
        return -1
    return max_elem - min(xs[:max_idx])


def detect_avg_color(image, contours):
    '''
    for 1 image, and all the contours in that image
    decide what the overall average colour is throughout the contours
    '''
    saved_b = 0
    saved_g = 0
    saved_r = 0
    whatwehadbefore = 0
    selected_countour = None

    """
    for contour in contours:
        avg_b = 0
        avg_g = 0
        avg_r = 0
        x, y, w, h = cv2.boundingRect(contour)
        total = w*h
        for i in range(x, x+w-1):
            for j in range(y, y+h-1):
                avg_b += image[j][i][0]
                avg_g += image[j][i][1]
                avg_r += image[j][i][2]

        avg_b = avg_b / total
        avg_g = avg_g / total
        avg_r = avg_r / total

        list_averages = [avg_b, avg_g, avg_r]

        difference = abs(max(list_averages) - min(list_averages))

        if(difference > whatwehadbefore):
            selected_countour = list_averages
    """
    fuking_fuk = []
    for contour in contours:

        x, y, w, h = cv2.boundingRect(contour)
        list_averages = [0, 0, 0]
        max_b = 0
        max_g = 0
        max_r = 0

        for i in range(x, x+w-1):
            for j in range(y, y+h-1):
                b = image[j][i][0]
                g = image[j][i][1]
                r = image[j][i][2]

                if (b > max_b):
                    max_b = b
                if (g > max_g):
                    max_g = g
                if (r > max_r):
                    max_r = r

        list_averages = [max_b, max_g, max_r]
        #print list_averages

        
        list_averages = selected_countour

    return selected_countour

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
