#!/usr/bin/env python
import roslib
roslib.load_manifest('ledcalibrator')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


current_image = 0

image_pub = rospy.Publisher("image_topic_2",Image)
bridge = CvBridge()



class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    global current_image

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # save in buffer
    current_image = cv_image



def my_callback(event):
  global current_image, bridge, image_pub, fgbg

  print 'Timer called at ' + str(event.current_real)
  for i in range(0,300):

    # capture background and foreground image

    #setAllOff()
    rospy.sleep(.5)
    bg_image = current_image

    #setLEDXOn()
    rospy.sleep(.5)
    fg_image = current_image


    # calculate diff image

    diff = np.absolute(fg_image.astype(int) - bg_image.astype(int))


    diff2 = diff.astype(np.uint8)

    gray_image = cv2.cvtColor(diff2, cv2.COLOR_BGR2GRAY)


    output = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)



    # find maximum location

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray_image)



    # report on console and image

    print("max_loc: ", max_loc)

    cv2.circle(output, max_loc, 10, 255);



    try:
      pass
      image_pub.publish(bridge.cv2_to_imgmsg(output, "bgr8"))
    except CvBridgeError as e:
      print(e)







def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rospy.Timer(rospy.Duration(1), my_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
