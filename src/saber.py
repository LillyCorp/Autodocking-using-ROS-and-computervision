#!/usr/bin/env python
#import arduinoserial
from pysabertooth import Sabertooth
from std_msgs.msg import String 
from geometry_msgs.msg import Twist, Pose
import rospy , time
import serial.tools.list_ports as port
#import pylcdlib
print "\nInit sabertooth....\n"

print "\nDetecting sabertooth....\n"
portlist = list(port.comports())
print portlist
address = ''
for p in portlist:
    print p
    if 'Sabertooth' in str(p):
        address = str(p).split(" ")
print "\nAddress found @"
print address[0]
speed1 = 0
speed2 = 0

saber = Sabertooth(address[0], baudrate=9600, address=128, timeout=0.1)
#lcd = pylcdlib.lcd(0x27,1)
#drive(number, speed)
#number: 1-2
#speed: -100 - 100
# while 1:
#     print('temperature [C]: {}'.format(saber.textGet('m2:gett')))
#     print('battery [mV]: {}'.format(saber.textGet('m2:getb')))
#     saber.text('m1:startup')
#     saber.text('m2:startup')
#     for speed in range(-100, 100, 20):
#             print speed
#             saber.drive(1, speed)
#             saber.drive(2, speed)

#             # format returned text
#             m1 = saber.textGet('A1:get\r\n').split()[1]
#             m2 = saber.textGet('m2:get').split()[1]
#             print('M1: {:6} M2: {:6}'.format(m1, m2))
#             time.sleep(1)


# while 1:
#     saber.text('m1:startup')
#     saber.text('m2:startup')
#     
#     #saber.drive(1,100)
#     print "balas"
#     pass



def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def callback(data):
    #print data
    saber.text('m1:startup')
    saber.text('1,p100')
    saber.text('md:1000\r\n')
    
    saber.text('m2:startup')
    saber.text('r1:1940\r\n')
    saber.text('r2:1940\r\n')
    speed = translate(data.linear.x,-1,1,-100,100)
    #speed = translate(data.linear.x,-1,1,-2047,2047)
    SPEED = 'md: {}\r\n'.format(speed)
    angle = translate(-data.angular.z,-1,1,100,-100)
    #angle = str(translate(-data.angular.z,-1,1,2047,-2047))
    ANGLE = 'mt: {}\r\n'.format(angle)
    #saber.text('m1:startup')
    #saber.drive(1,speed)
    #saber.drive(2,speed)
    if angle+speed > 100:
        speed1 = 100
    else :
        speed1 = angle+ speed
    if speed-angle < -100:
        speed2 = -100
    else :
        speed2 = speed-angle
    #saber.drive(1,speed)
    #saber.drive(2,speed)
        

    if(angle < 0):
        print "negative"
        saber.drive(1,speed2)
        saber.drive(2,speed1)
    elif (angle > 0):
        print "positive"
        saber.drive(1,speed2)
        saber.drive(2,speed1)
    

    #saber.text('m2:startup')
    #MD: 0\r\n 
    #print SPEED
    #print ANGLE
    
    #saber.text(SPEED)

    pass
    #print message

def sabertoothStatusCallback(data):
    print data
    temperature = ('T [C]: {}'.format(saber.textGet('m2:gett')))
    
    saber.textGet('T,start')
    set_position = ('P : {}'.format(saber.textGet('T,p45')))
    saber.textGet('1, home')
    
    battery = ('battery [mV]: {}'.format(saber.textGet('m2:getb'))) 
    print battery , temperature
    #lcd.lcd_write(0x0C) #Cursor uitschakelen.
    #lcd.lcd_write(0x01) #Scherm leegmaken.
    #lcd.lcd_puts("Hallo", 1) #Tekst voor LCD display lijn 1.
    #lcd.lcd_puts("  Wereld!", 2) #Tekst voor LCD display lijn 2.
    #lcd.lcd_backlight(1) #Achterverlichting aanzetten.
    


    
def listener():
    
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("/joy_teleop/cmd_vel", Twist, callback)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    #sabertoothStatusCallback("as")
    #rospy.Subscriber("/mastercmd_vel", Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()



