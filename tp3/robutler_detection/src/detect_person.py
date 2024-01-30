#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('robutler_detection')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    self.image_pub = rospy.Publisher("peron_detect",Image,queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    # my code here----------------------------------

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    vid_thresh = hsv_image.copy()

    # h s v value to segment blue for the jeans, to obtain other value use color_segmenter.py and a photo 
    lower_bound = np.array([100,150,0])
    upper_bound = np.array([228,255,150])
    
    # TODO lower_bound,upper_bound = color_segmenter()
    # # masking the image using in.range function
    vid_mask=cv2.inRange(vid_thresh,lower_bound, upper_bound)

    #find all the contours of the segmented mask
    #cnts,_ = cv2.findContours(vid_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    res = cv2.connectedComponentsWithStats(vid_mask.copy) #deteta as calças
    (total_labels, labels, stats, centroids) = res


    #checking if any countor is detected then ru the following statements
    if total_labels > 0:
        # sorting the contours to find biggest contour
        center = (centroids[label, 0], centroids[label, 1])
        
        start_point_x = stats[label, cv2.CC_STAT_LEFT]
        start_point_y = stats[label, cv2.CC_STAT_TOP]

        width = stats[label, cv2.CC_STAT_WIDTH]
        height = stats[label,cv2.CC_STAT_HEIGHT]

        start_point = (start_point_x, start_point_y)
        end_point = (start_point_x + width, start_point_y + height)

        #recortamos a zona das calças
        calças = vid_mask[start_point[1]: end_point[1], start_point[0]: end_point[0]] 

        element_shape = cv2.MORPH_ELLIPSE
        element_size = 7
        element = cv2.getStructuringElement(element_shape, (element_size, element_size))

        n = 3
        for i in range(n): #erodimos para ficar com as 2 pernas
            calças = cv2.erode(calças, element)
       
        #confirmamos se são as 2 pernas 
        res = cv2.connectedComponentsWithStats(vid_mask.copy)
        (total_labels, labels, stats, centroids) = res

        
        if total_labels <=2 :
            cv2.line(cv_image, (int(center[0]) - 10, int(center[1]) + 10), (int(center[0]) + 10, int(center[1]) - 10), (0, 0, 255), 1)
            cv2.line(cv_image, (int(center[0]) + 10, int(center[1]) + 10), (int(center[0]) - 10, int(center[1]) - 10), (0, 0, 255), 1)

    #-----------------------------------------------
    
    # cv2.imshow('Image', cv_image)
    # cv2.imshow("Mask window", vid_mask)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    

def main(args):
  rospy.init_node('detect', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)