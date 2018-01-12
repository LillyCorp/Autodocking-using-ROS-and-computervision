#!/usr/bin/env python


import sys
import rospy
import cv2 , math
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range



class image_converter:
  def translate(self, value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image ,queue_size=10)
    self.twist_pub = rospy.Publisher('mastercmd_vel' , Twist ,queue_size=10)

    # rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callback)
    self.command_sub = rospy.Subscriber("/mastercmd_vel", Twist, self.Commandcallback)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
    self.twist = Twist()

  def Commandcallback(self, data):
    pass
    #print "Manual control \n"

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)



    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    try:
      self.PoseEstimator(cv_image)
    except:
      pass
      print "Failed Pose estimator"
    try:
      self.laser_line_detector(cv_image)
    except:
      pass
      print "Falied line laser"

  def PoseEstimator(self, frame):
    # convert the frame to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)
    # find contours in the edge map
    # print cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    Area = 0
    # loop over the contours
    for c in cnts:
      # approximate the contour
      peri = cv2.arcLength(c, True)
      approx = cv2.approxPolyDP(c, 0.01 * peri, True)

      # ensure that the approximated contour is "roughly" rectangular
      if len(approx) >= 4 and len(approx) <= 6:
        # compute the bounding box of the approximated contour and
        # use the bounding box to compute the aspect ratio
        (x1, y1, w, h) = cv2.boundingRect(approx)
        # print x1
        # print y1

        aspectRatio = w / float(h)

        # compute the solidity of the original contour
        Area = []
        area = cv2.contourArea(c)
        Area = max(area, Area)

        hullArea = cv2.contourArea(cv2.convexHull(c))
        solidity = area / float(hullArea)

        # compute whether or not the width and height, solidity, and
        # aspect ratio of the contour falls within appropriate bounds
        keepDims = w > 25 and h > 25
        keepSolidity = solidity > 0.9
        keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2

        # ensure that the contour passes all our tests
        if keepDims and keepSolidity and keepAspectRatio:
          # draw an outline around the target and update the status
          # text
          cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
          status = "Target(s) Acquired"
          # This will give you the Pixel location of the rectangular box
          rc = cv2.minAreaRect(approx[:])
          # print rc,'rc'
          box = cv2.boxPoints(rc)
          pt = []
          for p in box:
            val = (p[0], p[1])
            pt.append(val)
            # print pt,'pt'
            cv2.circle(frame, val, 5, (200, 0, 0), 2)
          #print pt
          M = cv2.moments(approx)
          (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
          (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
          (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
          cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
          cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
          #print "cX", cX
          cv2.putText(frame, str(cX), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 2, 100)



          # 2D image points. If you change the image, you need to change vector
          image_points = np.array([pt], dtype="double")
          # print image_points
          size = frame.shape
          # 3D model points.
          model_points = np.array([
            (0.0, 0.0, 0.0),  # Rectangle center
            (0.0, 13.6, 0.0),  #
            (13.6, 13.6, 0.0),  #
            (13.6, 0.0, 0.0),  #

          ])

          # Camera intrinsic parameters

          focal_length = size[1]
          center = (size[1] / 2, size[0] / 2)
          camera_matrix = np.array(
            [[focal_length, 0, center[0]],
             [0, focal_length, center[1]],
             [0, 0, 1]], dtype="double"
          )

          # print "Camera Matrix :\n {0}".format(camera_matrix)
          criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

          dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
          (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix,
                                                                        dist_coeffs, criteria)

          rotationMatrix = np.zeros((3, 3), np.float32)

          # print "Rotation Vector:\n {0}".format(rotation_vector)
          print "Translation Vector:\n {0}".format(translation_vector)
          print translation_vector[2]
          cv2.putText(frame, str(round(translation_vector[2],2)), (cX, cY+50), cv2.FONT_HERSHEY_SIMPLEX, 2, 100)
          cv2.imshow("Image window", frame)

          # Rodrigues to convert it into a Rotation vector
          #print rotation_vector
          dst, jacobian = cv2.Rodrigues(rotation_vector)
          #print "Rotation matrix:\n {0}".format(dst)
          # sy = math.sqrt(dst[0, 0] * dst[0, 0] + dst[1, 0] * [1, 0])
          A = dst[2, 1] * dst[2, 2]
          B = dst[1, 0] * dst[0, 0]
          # C = -dst[2,0]*sy

          if success:
          #print cX
            self.twist.linear.x = self.translate(translation_vector[2], 45 ,200 ,-.1, 1 )
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = self.translate(cX, 50 ,600,-1, 1 )

            #print "Twist values are " , self.twist
            try:
              self.twist_pub.publish(self.twist)
              print self.twist
            except:
              print "failed to publish"


          # Eular angles

            Theta_x = math.degrees(math.atan(A) * 2)
            #Theta_y = math.degrees(math.atan(C)*2)
            Theta_z = math.degrees(math.atan(B) * 2)
            #print "Roll axis:\n {0}".format(Theta_x)
            #print "Yaw axis:\n {0}".format(Theta_z)
          cv2.imshow("Pose estimator", frame)
          cv2.waitKey(1)



  def laser_line_detector(self,frame):
    Gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    v = np.median(frame)
    # apply automatic Canny edge detection using the computed median
    sigma = 0.33
    lower = int(max(100, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edges = cv2.Canny(Gray , lower, upper)
    #cv2.imshow("Edges",edges)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_red = np.array([130, 10, 150])
    upper_red = np.array([255, 2200, 255])

    # lower_green = np.array([100, 25, 20])
    # upper_green = np.array([200, 159, 169])
    # green_mask = cv2.inRange(hsv, lower_green, upper_green) # I have the Green threshold image.

    # Threshold the HSV image to get only blue colors
    blue_mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = blue_mask

    # Bitwise-AND mask and original image
    kernel = np.ones((5, 5), np.uint8)
    erosion = cv2.erode(frame, kernel, iterations=1)
    th3 = cv2.adaptiveThreshold(mask, 150, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 2)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    ret, thresh = cv2.threshold(mask, 50, 200, cv2.THRESH_BINARY)
    opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    npimg = np.asarray(thresh)
    pixel = np.argwhere(npimg > 20)
    #print pixel

    cv2.imshow('frame', frame)
    cv2.waitKey(1)
    #cv2.imshow('mask', th3)
    #cv2.imshow('res', res)


def main(args):
  ic = image_converter()
  rospy.init_node('listner', anonymous=True)
  rate = rospy.Rate(rospy.get_param('~hz', 10))
  try:
    rospy.spin()
    rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

