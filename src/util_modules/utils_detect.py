import cv2

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
    return (avg_b/total, avg_g/total, avg_r/total)

def euclidian_colour_diff(colour1, colour2):
    gdiff = colour2[0] - colour1[0]
    bdiff = colour2[1] - colour1[1]
    rdiff = colour2[2] - colour1[2]

    return (gdiff + bdiff + rdiff)/3
