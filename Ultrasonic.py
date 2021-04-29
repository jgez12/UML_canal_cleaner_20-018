#!/usr/bin/env python

## import stuff:
import rospy
from std_msgs.msg import Float32
import smbus                    
from time import sleep 


## declare stuff:
ADDRESS = 0x38 #(use 7-bit addressing)
RANGE_CMD = 0x51 #81
bus = smbus.SMBus(1) 

if __name__ == '__main__':
    pub = rospy.Publisher('LFUS', Float32, queue_size=10)
    rospy.init_node('Ultrasonic', anonymous=True)
    rate=rospy.Rate(10)
    
    while not rospy.is_shutdown():

        ###### PUT MAIN CODE HERE #####
        while True:
            bus.write_byte_data(ADDRESS, 0, RANGE_CMD)
            sleep(1)
            #read 2 bytes
            dist = bus.read_i2c_block_data(ADDRESS, 1, 2)
            print dist[1]/2.5 # turn into inches
        ###############################

        pub.publish(dist[1]/2.5)
        rate.sleep()

