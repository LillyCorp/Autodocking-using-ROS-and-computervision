#!/usr/bin/env python

from pysabertooth import Sabertooth
from std_msgs.msg import String 
from geometry_msgs.msg import Twist, Pose
import rospy , time



saber = Sabertooth('/dev/ttyACM0', baudrate=38400, address=128, timeout=0.1)





def callback(data):
    
    temperature = ('T [C]: {}'.format(saber.textGet('m2:gett')))
    battery = ('battery [mV]: {}'.format(saber.textGet('m2:getb')))
    current = ('current [mA]: {}'.format(saber.textGet('m2:getc')))
    print temperature
    print battery
    print current
    


    
def listener():
	
    pub = rospy.Publisher('/Junkbot/info', String, queue_size=10) 
    rospy.init_node('talker', anonymous=True)
	
    rospy.Subscriber("/junkbot/cmd_vel", Twist, callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    listener()



