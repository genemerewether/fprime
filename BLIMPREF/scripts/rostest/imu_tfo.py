#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Imu
#import future

pub = None
    
def callback(data):
    if pub is None:
        #print("publisher not initialized")
        return
    msg = Imu()
    msg.header = data.header
    msg.linear_acceleration.x = data.linear_acceleration.y
    msg.linear_acceleration.y = data.linear_acceleration.z
    msg.linear_acceleration.z = data.linear_acceleration.x
    
    msg.angular_velocity.x = data.angular_velocity.y
    msg.angular_velocity.y = data.angular_velocity.z
    msg.angular_velocity.z = data.angular_velocity.x
    pub.publish(msg)

def republish():
    global pub
    rospy.init_node('imu_repub', anonymous=True)
    pub = rospy.Publisher('ext_imu', Imu, queue_size=100)
    rospy.Subscriber('imu/imu', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        republish()
    except rospy.ROSInterruptException:
        pass
