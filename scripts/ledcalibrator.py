#!/usr/bin/env python
import roslib
roslib.load_manifest('ledcalibrator')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

count = 0
MARKERS_MAX = 500

import numpy as np

sys.path.append('../../pyledstrip')
import pyledstrip

current_ros_image = None
current_image = np.array([])

image_pub = rospy.Publisher("image_topic_output",Image)
image_pub2 = rospy.Publisher("image_topic_bg_image",Image)
image_pub3 = rospy.Publisher("image_topic_fg_image",Image)
bridge = CvBridge()

marker_publisher = rospy.Publisher("visualization_marker_array", MarkerArray)

strip_pixel_coordinates = None

DISTANCE_BETWEEN_TWO_LEDS = 100/60

delay = 0.3


class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    global current_image, current_ros_image

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    # save in buffer
    current_ros_image = data
    current_image = cv_image


def run_calibration_sweep(total_ids = 300, strip_resolution = 5, accumulated_frames = 3, delay = 0.5):
  '''runs one sweep on the lightstrip and returns the detected coordinates'''
  global current_image, bridge, image_pub, image_pub2, image_pub3 

  rospy.loginfo("running calibration sweep ...")

  coordinates = {}

  if current_image.size == 0:
    rospy.logwarn("no image in the buffer. aborting.")
    return

  for i in range(0, total_ids, strip_resolution):

    # capture background and foreground image

    #setAllOff()
    pyledstrip.clear()
    pyledstrip.transmit()

    rospy.sleep(delay)
    bg_image = current_image.astype(int)

    #setLEDXOn()
    pyledstrip.set_rgb(i,255,255,255)
    pyledstrip.transmit()


    ACCUMULATE_FRAMES = accumulated_frames

    fg_image = np.array([])
    
    rospy.sleep(delay)
    fg_image = current_image.astype(int)

    for j in range(1,ACCUMULATE_FRAMES):
      rospy.sleep(delay)
      fg_image = fg_image + current_image.astype(int)

    fg_image = fg_image / ACCUMULATE_FRAMES


    # second background image
    pyledstrip.clear()
    pyledstrip.transmit()

    rospy.sleep(delay)
    bg_image2 = current_image.astype(int)


    # calculate diff image

    diff = np.maximum(fg_image - bg_image - bg_image2, 0)


    diff2 = diff.astype(np.uint8)

    gray_image = cv2.cvtColor(diff2, cv2.COLOR_BGR2GRAY)


    output = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)



    # find maximum location

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray_image)



    # report on console and image

    coordinates[i] = max_loc

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

  return coordinates

 

def estimate_metric_coordinates(pixel_coordinates):
  '''estimate metric coordinates from pixel coordinates using a distance assumption between two LEDs'''
  
  rospy.logwarn("estimate_metric_coordinates not implemented yet. returning pixel coordinates.")
  return pixel_coordinates

  '''TODO'''


def verify_coordinates(coordinates):
  '''verifies the coordinates by calculating the min and max distance between points'''
  return False


def visualize_coordinates(coordinates):
  '''visualizes the coordinates using ROS markers'''
  global count

  markerArray = MarkerArray()

  for key, coordinate in coordinates.iteritems():
    marker = Marker()
    marker.header.frame_id = "/camera_pixel"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(coordinate[0]) / 100.
    marker.pose.position.y = 0
    marker.pose.position.z = float(coordinate[1]) / 100.

    # We add the new marker to the MarkerArray, removing the oldest
    # marker from it when necessary
    if (count > MARKERS_MAX):
      markerArray.markers.pop(0)

    markerArray.markers.append(marker)

  # Renumber the marker IDs
  id = 0
  for m in markerArray.markers:
    m.id = id
    id += 1

  marker_publisher.publish(markerArray)

  count += 1

  return True


def detect_led_on(led_id, bg_image, image):
  '''detects if led x is on using the led coordinates.
     we currently run the full picture background substraction and assume detection if distance is lower than threshold.
     We should use a region of interest and a light detection at the known position instead'''
  global strip_pixel_coordinates

  led_coordinate = strip_pixel_coordinates[led_id]

  # calculate diff image
  diff = np.maximum(image - bg_image, 0)

  diff2 = diff.astype(np.uint8)

  gray_image = cv2.cvtColor(diff2, cv2.COLOR_BGR2GRAY)

  output = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

  # find maximum location

  min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray_image)

  distance_to_led = np.linalg.norm(np.array(led_coordinate) - np.array(max_loc))

  rospy.loginfo("Distance to LED: %d", distance_to_led)

  cv2.circle(output, max_loc, 10, 255);

  try:
    pass
    image_pub.publish(bridge.cv2_to_imgmsg(output, "bgr8"))
    image_pub2.publish(bridge.cv2_to_imgmsg(bg_image.astype(np.uint8), "bgr8"))
    image_pub3.publish(bridge.cv2_to_imgmsg(image.astype(np.uint8), "bgr8"))
  except CvBridgeError as e:
    print(e)

  '''dirty: hard-coded threshold'''
  return (distance_to_led < 10)



def estimate_delay(led_id):
  '''estimates the time delay from sending an led activation to the feedback at the camera'''
  global current_ros_image

  rospy.loginfo('estimating delay ...')

  pyledstrip.clear()
  pyledstrip.transmit()

  rospy.sleep(.5)
  bg_image = current_image.astype(int)
  old_seq = current_ros_image.header.seq

  #setLEDXOn()
  pyledstrip.set_rgb(led_id,255,255,255)

  send_time = rospy.get_rostime()
  pyledstrip.transmit()

  rospy.loginfo("Sent led on at %i %i", send_time.secs, send_time.nsecs)

  # hacky image-subscription
  rcv_time = 0
  for i in range(0,100):
    ros_image = current_ros_image
    if ros_image.header.seq == old_seq:
      rospy.sleep(0.01)
      continue

    led_on = detect_led_on(led_id, bg_image, current_image)
    if led_on:
      rcv_time = ros_image.header.stamp
      rospy.loginfo("Rcv led on at %i %i", rcv_time.secs, rcv_time.nsecs)
      break

    old_seq = ros_image.header.seq


  if not led_on:
    rospy.logerr("no LED detected")
    return False, 0

  delay = rcv_time - send_time

  seconds_delay = float(delay.secs) + float(delay.nsecs) / 1000000000.

  rospy.loginfo("Delay is %f s", seconds_delay)

  return True, seconds_delay


def batch_estimate_delay(led_id, num_tests = 10):
  max_delay = 0
  overall_success = False
  for i in range(0,num_tests):
    success, new_delay = estimate_delay(led_id)
    if success:
      overall_success = success
      max_delay = max(new_delay, max_delay)

  return overall_success, max_delay





def my_callback(event):
  global current_image, bridge, image_pub, image_pub2, image_pub3, strip_pixel_coordinates, delay

  print 'Timer called at ' + str(event.current_real)

  coordinates = run_calibration_sweep(300,5,1,delay)

  strip_pixel_coordinates = coordinates

  metric_coordinates = estimate_metric_coordinates(coordinates)

  print(coordinates)
  print(metric_coordinates)

  visualize_coordinates(coordinates)

  # This can be used to estimate the delay automatically

  # success, new_delay = batch_estimate_delay(10)
  #
  # if success:
  #   new_delay = new_delay * 1.1
  # else:
  #   new_delay = delay + 0.1
  #
  # rospy.loginfo("updating delay to %f", new_delay)
  # delay = new_delay






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
