#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import Imu
import smbus
import math
from math import sin, cos, pi
import tf 
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(reg):
    return bus.read_byte_data(address, reg)

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect

# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)

def talker():
    pub = rospy.Publisher('example/imu', Imu, queue_size=50)
    rospy.init_node('imu_node', anonymous=True)
    start_time = rospy.Time.now()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    roll_ = 0
    yaw_ = 0
    pitch_ = 0
    rate = rospy.Rate(10) # 10hz while not rospy.is_shutdown(): odom = Odometry()
    while not rospy.is_shutdown():
        gyroskop_xout = read_word_2c(0x43)
        gyroskop_yout = read_word_2c(0x45)
        gyroskop_zout = read_word_2c(0x47)

        roll = gyroskop_xout / 131
        pitch = gyroskop_yout / 131
        yaw = gyroskop_zout / 131

        roll_ = roll_+roll
        yaw_=yaw_+yaw
        pitch_=pitch_+pitch

        quat = tf.transformations.quaternion_from_euler(roll_,pitch_,yaw_)

        accleration_xout = read_word_2c(0x3b)
        accleration_yout = read_word_2c(0x3d)
        accleration_zout = read_word_2c(0x3f)

        a_x = accleration_xout / 1535
        a_y = accleration_yout / 1535
        a_z = accleration_zout / 1535

        i = Imu()
        i.header.stamp = rospy.Time.now()
        i.header.frame_id = 'base_link'
        i.orientation.x = quat[0]
        i.orientation.y = quat[1]
        i.orientation.z = quat[2]
        i.orientation.w = quat[3]
        i.angular_velocity.x = roll
        i.angular_velocity.y = pitch
        i.angular_velocity.z = yaw
        i.linear_acceleration.x = a_x
        i.linear_acceleration.y = a_y
        i.linear_acceleration.z = a_z
        pub.publish(i)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
