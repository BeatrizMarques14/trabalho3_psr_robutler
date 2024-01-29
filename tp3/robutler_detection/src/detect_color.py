#!/usr/bin/env python3
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
import argparse
from std_msgs.msg import Bool


class image_converter:

  def __init__(self, color):

    self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
    self.result_pub = rospy.Publisher("/object_detection_result", Bool, queue_size=1)

    # Convert color string to lower case for case-insensitive comparison
    self.color = color.lower()

    # Define color bounds based on the specified color
    self.color_bounds = self.get_color_bounds()

  def get_color_bounds(self):
    # Define default bounds for unknown colors
    lower_bound = np.array([0, 0, 0])
    upper_bound = np.array([255, 255, 255])

    # Update bounds based on the specified color
    if self.color == "violet":
        lower_bound = np.array([130, 50, 50])
        upper_bound = np.array([160, 255, 255])
    elif self.color == "blue":
        # Add bounds for the "blue" color
        lower_bound = np.array([110, 100, 100])
        upper_bound = np.array([130, 255, 255])
    # Add more colors as needed

    return lower_bound, upper_bound

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    (rows, cols, channels) = cv_image.shape
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Use color bounds based on the specified color
    lower_bound, upper_bound = self.color_bounds

    vid_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    cnts, _ = cv2.findContours(vid_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    object_found = False
    if len(cnts) > 0:
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        M = cv2.moments(cnt)
        if M['m00'] != 0:
          object_found = True
          center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
          cv2.line(cv_image, (int(center[0]) - 10, int(center[1]) + 10),
                   (int(center[0]) + 10, int(center[1]) - 10), (0, 0, 255), 2)
          cv2.line(cv_image, (int(center[0]) + 10, int(center[1]) + 10),
                   (int(center[0]) - 10, int(center[1]) - 10), (0, 0, 255), 2)
    


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.result_pub.publish(object_found)
    except CvBridgeError as e:
      print(e)


def main(args):
  parser = argparse.ArgumentParser(description='Detect color in images.')
  parser.add_argument('color', type=str, help='Color to detect (e.g., violet, blue)')

  args = parser.parse_args()
  
  rospy.init_node('detect', anonymous=True)
  ic = image_converter(args.color)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
