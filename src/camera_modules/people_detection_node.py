from imutils.object_detection import non_max_suppression
import cv2
import camera_node
import numpy
import imutils

def detect_angle_to_person(image,rectangle):
    
    fov = 90    

    mid_image = image.shape[1]/2.0
    centre = ((rectangle[0][0] + rectangle[1][0])/2)
    if centre > mid_image:
        #print 
        angle = ((fov/2.0)/mid_image) * (centre - mid_image)
    else:
        angle = ((fov/2.0)/mid_image) * (centre - mid_image)   
    print "angle: ", angle
    return angle

def detect_people(image):

    image = imutils.resize(image, width=min(400, image.shape[1]))

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    orig = image.copy()
    
    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        padding=(8, 8), scale=1.05)
    
    # draw the original bounding boxes
    for (x, y, w, h) in rects:
        cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
    	print "angle: ", detect_angle_to_person(image, ((x, y), (x + w, y + h)))
    
    
    return orig
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    """
    rects = numpy.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    
    # draw the final bounding boxes
    for (xA, yA, xB, yB) in pick:
        cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
    """

def whatever(camera_checker):
    cap = cv2.VideoCapture(1)

    data = None
    while True:
        # Get the current frame
        ret, frame = cap.read()

        # Apply the method
        data = camera_checker(frame)

        # Make frame
        cv2.imshow('frame', data)

        # Publish frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Once done, destroy all windows
    # We can move this destroy and the VideoCapture at the beginning
    # to some sort of an init method later on
    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    print "start"
    whatever(detect_people)
