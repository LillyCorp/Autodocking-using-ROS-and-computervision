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
    
    self.twist_pub = rospy.Publisher('/pocketbot/cmd_vel' , Twist ,queue_size=10)

    # rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callback)
    self.pose_sub = rospy.Subscriber("/pocketbot/face", Twist, self.callback)
    self.twist = Twist()	
    

  def callback(self,data):
    print data
    self.twist.linear.x = data.linear.x
    self.twist.linear.y = 0
    self.twist.linear.z = 0
    self.twist.angular.x = 0
    self.twist.angular.y = 0
    self.twist.angular.z = data.linear.z

    try:
   	#self.twist_pub.publish(self.twist)
        #print self.twist
	pass
    except:
        print "publish failed"
   

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

