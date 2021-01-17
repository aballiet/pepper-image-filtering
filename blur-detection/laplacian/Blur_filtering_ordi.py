import rospy
import math
import time
import json
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random
from imutils    import paths
from matplotlib import pyplot as plt
import cv2
import os
import shutil

tab_log = {}
tab_log["image_id_to_chars"]={}
vitesse = 0.
vitesse_ang = 0. 
counter = 0
total = 0
threshold = 1500
crop_factor = 1

def callback(data):
    global tab_log
    #print('salut')
    global counter
    global total
    lock = threading.Lock()
    global threshold
    global crop_factor
    cvBridge = CvBridge()

    image = cvBridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    #print('i got an image !')

    height, width = len(image), len(image[0])
    image_reserve = image

    # cropping image
    y1, y2 = int(height*crop_factor/2.), height - int((height*(1-crop_factor))/2.)
    x1, x2 = int(width*crop_factor/2.), width - int((width*(1-crop_factor))/2.)

    #print("height {}, width {}".format(height, width))
    #print("y1, y2 : ({}, {}), x1, x2 : ({}, {})".format(y1, y2, x1, x2))

    image = image[y1: y2, x1:x2]

    # convert to black and white
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #print('image cropped and converted !')

    # compute laplacian variation
    fm = cv2.Laplacian(image, cv2.CV_64F).var()

    print('The Laplacian of this picture is {} with a threshold of {}'.format(fm, threshold))

    # if the focus measure is less than the supplied threshold,
    # then the image should be considered "blurry"

    if fm > threshold:
        counter += 1
	print('published !')
        image_pub.publish(cvBridge.cv2_to_imgmsg(image_reserve))

    total += 1

    #print("Detected {} blurred images out of {}".format(counter, total))
    
    lock.acquire()
    tab_log["image_id_to_chars"][data.header.seq]={"vitesse":vitesse, "vitesse_ang":vitesse_ang, "variance_laplacien": fm}
    lock.release()

    #return counter / total

def callback2(data):
    vitesse = math.sqrt(math.pow((data.twist.twist.linear.x),2)+math.pow((data.twist.twist.linear.y),2))
    vitesse_ang = data.twist.twist.angular.z
    



if __name__ == '__main__':
    rospy.init_node('Blur_filtering', anonymous=True, disable_signals=True)
    image_pub = rospy.Publisher('/pepper_robot/camera/front/image_blurry_filtered', Image, queue_size = 100)
    rospy.Subscriber('/pepper_robot/camera/front/image_raw', Image, callback)
    rospy.Subscriber('/pepper_robot/odom', Odometry, callback2)
    while True:
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            ratio = counter/total
            lock.acquire()
            tab_log["sharp_ratio"] = ratio
            with open('data.txt', 'w') as outfile:
                json.dump(tab_log, outfile)
            lock.release()
            break

File "Blur_filtering.py", line 98, in <module>
    json.dump(tab_log, outfile)
  File "/tmp/gentoo/usr/lib/python2.7/json/__init__.py", line 189, in dump
    for chunk in iterable:
  File "/tmp/gentoo/usr/lib/python2.7/json/encoder.py", line 434, in _iterencode
    for chunk in _iterencode_dict(o, _current_indent_level):
  File "/tmp/gentoo/usr/lib/python2.7/json/encoder.py", line 408, in _iterencode_dict
    for chunk in chunks:
  File "/tmp/gentoo/usr/lib/python2.7/json/encoder.py", line 364, in _iterencode_dict
    for key, value in items:
RuntimeError: dictionary changed size during iteration




if __name__ == '__main__':
    rospy.init_node('Blur_filtering', anonymous=True)
    image_pub = rospy.Publisher('/pepper_robot/camera/front/image_blurry_filtered', Image, queue_size = 100)
    rospy.Subscriber('/pepper_robot/camera/front/image_raw', Image, callback)
    rospy.Subscriber('/pepper_robot/odom', Image, callback)
    rospy.spin()
