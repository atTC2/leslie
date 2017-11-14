#!/usr/bin/env python

from imutils.object_detection import non_max_suppression
import cv2
import camera_node
import numpy
import imutils
import json
import cv2
#import subprocess
import rospy
#from threading import Thread
#from Queue import Queue
#from datetime import datetime
from std_msgs.msg import String
from state_machine import states, actions

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
    #for (x, y, w, h) in rects:
    #    cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
    	
    #detect_angle_to_person(image, ((x, y), (x+w, y + h)))
    
    angle = 0
    lost = True
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    if len(rects) !=0:
        rects = numpy.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    
        maxX = 0 
        check = 0
    # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(orig, (xA, yA), (xB, yB), (0, 255, 0), 2)
            check = abs(xA-xB)
            if check > maxX:
                maxX = check
                angle = detect_angle_to_person(orig, ((xA, yA), (xB, yB)))
        print "angle: ", angle
        lost = False
    
    return orig, angle, lost

def get_distance():
    #------------------------------
    # SEB CODE
    #------------------------------
    return 1

def whatever(camera_checker):
    cap = cv2.VideoCapture(1)

    close_threshold = 1500

    data = None
    counter = 0
    while True:
        # Get the current frame
        ret, frame = cap.read()

        # Apply the method
        data, angle, lost = camera_checker(frame)
        distance = get_distance()
        # Make frame
        cv2.imshow('frame', data)
        if lost:
            counter += 1
        
        if counter == 600:
		if distance < close_threshold: 
		   return True
		else:
		   return False

        waypoint_pub.publish(json.dumps({'angle':angle,'distance':distance}))

        # Publish frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Once done, destroy all windows
    # We can move this destroy and the VideoCapture at the beginning
    # to some sort of an init method later on
    cap.release()
    cv2.destroyAllWindows()

def callback(state_msg):
    """
    Handles new state information, and watches for faces if the state is right
    :param state_msg: The new state information
    :type state_msg: std_msgs.msg.String
    """
    #global SLEEP_TIME

    print "state_msg: ", state_msg
    state = json.loads(state_msg.data)
    action = {}
    if state['id'] == states.ALARM:
        result = whatever(detect_people)
        print "result: ", result
        if result:
            print "caught " 
            pub.publish(json.dumps({'id': actions.CAUGHT_THIEF, 'data': state['data']}))
        else:
            print "lost "
            pub.publish(json.dumps({'id': actions.LOST_THIEF, 'data': state['data']}))



# ROS node stuff
rospy.init_node('follow_node')
rospy.Subscriber('/state', String, callback, queue_size=10)
pub = rospy.Publisher('/action', String, queue_size=10)
waypoint_pub = rospy.Publisher('/waypoint', String, queue_size=1)
rospy.Subscriber('/camera/depth/image_raw', Image, get_distance)
callback(json.dumps("Hi"))
#rospy.spin()
