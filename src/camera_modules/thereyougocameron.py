import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import struct
import scipy.ndimage as ndimage


last_image = None
threshold = 0.3
alarm = False
ith_frame = 0
threshold_frame = 20
last_images = []

def get_distance(img):

    global last_image, alarm, ith_frame, threshold_frame

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "16UC1")

    cv_image = np.array(cv_image, dtype=np.float32)
    unnorm_cv_image = cv_image[200:480, 100:620, 0:1]
  
    unnorm_cv_image = ndimage.gaussian_filter(unnorm_cv_image, sigma=(5, 5, 0), order=0)

    unnorm_cv_image = cv2.normalize(unnorm_cv_image, unnorm_cv_image, 0, 1, cv2.NORM_MINMAX)

    once = True
    values = []

    saved_full_image = unnorm_cv_image.copy()
    saved_full_image = cv2.cvtColor(saved_full_image, cv2.COLOR_GRAY2RGB)
    if ith_frame == threshold_frame:
	cropped = unnorm_cv_image
        alarm_array = []
	if len(last_images) >= 7:
            for image in last_images:
                diff_array = np.subtract(image, cropped)
	        diff_array = ndimage.gaussian_filter(diff_array, sigma=(5, 5, 0), order=0)

                #print(max(diff_array.flatten()))
                #print(min(diff_array.flatten()))
                alarm_array.append(any(abs(i) > threshold for i in diff_array.flatten()))
                for i in range(0, len(diff_array)):
                    for j in range(0, len(diff_array[i])):
                        if(abs(diff_array[i][j]) > threshold):
                            cv2.rectangle(saved_full_image, (j, i), (j, i), (0, 0, 255), 2)
        else:
            last_images.append(cropped)
            if(len(last_images) >= 7):
                print ("Locked")
	#print alarm_array 
        max_number = 4
        if (alarm_array.count(True) >= max_number):
            print "ALARM!"
        else:
            print "fuck you cameron"

        ith_frame = 0
        cv2.imshow('img', saved_full_image)
        cv2.waitKey(1)   
    else:
       ith_frame += 1
  
"""
    if ith_frame == threshold_frame:
	cropped = unnorm_cv_image
	if last_image is not None:
            diff_array = np.subtract(last_image, cropped)
	    diff_array = ndimage.gaussian_filter(diff_array, sigma=(5, 5, 0), order=0)
            print (max(diff_array.flatten()))
            print (min(diff_array.flatten()))
            alarm = any(abs(i) > threshold for i in diff_array.flatten())
        else:
            last_image = cropped
	print alarm 
        ith_frame = 0
    else:
       ith_frame += 1
"""

rospy.init_node('ho')
rospy.Subscriber('/camera/depth/image_raw', Image, get_distance)
rospy.spin()
