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

sys.path.append('../../pyledstrip')
import pyledstrip

current_image = np.array([])

image_pub = rospy.Publisher("image_topic_output",Image)
image_pub2 = rospy.Publisher("image_topic_bg_image",Image)
image_pub3 = rospy.Publisher("image_topic_fg_image",Image)
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
  global current_image, bridge, image_pub, image_pub2, image_pub3, fgbg

  print 'Timer called at ' + str(event.current_real)
  
  if current_image.size == 0:
    return

  for i in range(0,300,10):

    # capture background and foreground image

    #setAllOff()
    pyledstrip.clear()
    pyledstrip.transmit()

    rospy.sleep(.1)
    bg_image = current_image.astype(int)

    #setLEDXOn()
    pyledstrip.set_rgb(i,255,255,255)
    pyledstrip.transmit()


    ACCUMULATE_FRAMES = 3

    fg_image = np.array([])
    
    rospy.sleep(.1)
    fg_image = current_image.astype(int)

    for j in range(1,ACCUMULATE_FRAMES):
      rospy.sleep(.1)
      fg_image = fg_image + current_image.astype(int)

    fg_image = fg_image / ACCUMULATE_FRAMES


    # second background image
    pyledstrip.clear()
    pyledstrip.transmit()

    rospy.sleep(.1)
    bg_image2 = current_image.astype(int)


    # calculate diff image

    diff = np.maximum(fg_image - bg_image - bg_image2, 0)


    diff2 = diff.astype(np.uint8)

    gray_image = cv2.cvtColor(diff2, cv2.COLOR_BGR2GRAY)


    output = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)



    # find maximum location

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray_image)



    # report on console and image

    print("{0}: {2}, ".format(i, max_loc[0], max_loc[1]))
    rospy.loginfo("%d: %d, ", i, max_loc[1])

    cv2.circle(output, max_loc, 10, 255);



    try:
      pass
      image_pub.publish(bridge.cv2_to_imgmsg(output, "bgr8"))
      image_pub2.publish(bridge.cv2_to_imgmsg(bg_image.astype(np.uint8), "bgr8"))
      image_pub3.publish(bridge.cv2_to_imgmsg(fg_image.astype(np.uint8), "bgr8"))
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
