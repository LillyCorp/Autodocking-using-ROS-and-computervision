#!/usr/bin/env python
#import arduinoserial
import serial
import serial.tools.list_ports as port
import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist, Pose


portlist = list(port.comports())
address = ''
for p in portlist:
	print p
	if 'CP2102' in str(p):
		address = str(p).split(" ")
print address[0]
arduino = serial.Serial(address[0], 9600, 8.0)






def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.linear)
    try:
        arduino.write(" ")
    except:
        portlist = list(port.comports())
        address = ''
        for p in portlist:
                print p
                if 'CP2102' in str(p):
                        address = str(p).split(" ")
        print address[0]
        arduino = serial.Serial(address[0], 9600, 8.0)


    print data.linear.x
    arduino.write('D:'+str(translate(data.linear.x,-1,1,-2047,2047)) +"\r\n")
    arduino.write('T:'+str(translate(data.angular.z,-1,1,2047,-2047))+"\r\n")
    message = arduino.read()
    print message
    
def listener():
	
	
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callback)
    rospy.Subscriber("/cmd_vel", Twist, callback)	
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



