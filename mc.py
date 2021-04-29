#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from time import sleep
 
#///////////////// Define Motor Driver GPIO Pins /////////////////
# Motor A, Left Side GPIO CONSTANTS
PWM_DRIVE_LEFT = 21	# ENA - H-Bridge enable pin, Rpi-40
FORWARD_LEFT_PIN = 19	# N1 - Forward Drive, Rpi-35
REVERSE_LEFT_PIN = 26	# N2 - Reverse Drive, Rpi-37

# Motor B, Right Side GPIO CONSTANTS
PWM_DRIVE_RIGHT = 20	# ENB - H-Bridge enable pin, Rpi-38
FORWARD_RIGHT_PIN = 13	# N3 - Forward Drive, Rpi-33
REVERSE_RIGHT_PIN = 6	# N4 - Reverse Drive, Rpi-31
 
# Initialise objects for H-Bridge GPIO PWM pins
# Set initial duty cycle to 0 and frequency to 1000
driveLeft = PWMOutputDevice(PWM_DRIVE_LEFT, True, 0, 1000)
driveRight = PWMOutputDevice(PWM_DRIVE_RIGHT, True, 0, 1000)
 
# Initialise objects for H-Bridge digital GPIO pins
forwardLeft = PWMOutputDevice(FORWARD_LEFT_PIN)
reverseLeft = PWMOutputDevice(REVERSE_LEFT_PIN)
forwardRight = PWMOutputDevice(FORWARD_RIGHT_PIN)
reverseRight = PWMOutputDevice(REVERSE_RIGHT_PIN)


########################################
########################################
########################################
def cmdIMU(msg)
    value = msg.data

def cmdLFUS(msg)
    value = msg.data

def cmdVelCB(twist):
	left_wheel_data = 5*twist.linear.x - twist.angular.z
	right_wheel_data = 5*twist.linear.x + twist.angular.z

#	print right_wheel_data, left_wheel_data

	if(right_wheel_data >= 0):
		forwardRight.value = True
		reverseRight.value = False
		driveRight.value = min(abs(right_wheel_data),1)
	else:
		forwardRight.value = False
		reverseRight.value = True
		driveRight.value = min(abs(right_wheel_data),1)


        if(left_wheel_data >= 0):
		forwardLeft.value = True
		reverseLeft.value = False
		driveLeft.value = min(abs(left_wheel_data),1)
	else:
		forwardLeft.value = False
		reverseLeft.value = True
		driveLeft.value = min(abs(left_wheel_data),1)

#	print driveLeft.value, driveRight.value
   
########################################
########################################
########################################


    
if __name__ == '__main__':
    print "start"
    rospy.init_node('motor', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, cmdVelCB)
    rospy.Subscriber('IMU', Float32, cmdIMU)
    rospy.Subscriber('LFUS', Float32, cmdLFUS)
    rospy.spin()
