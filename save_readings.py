#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    """
    Callback function to process incoming IMU data.
    """
    rospy.loginfo("Received IMU Data:")
    rospy.loginfo("Header:")
    rospy.loginfo(f"  Frame ID: {msg.header.frame_id}")
    rospy.loginfo(f"  Timestamp: {msg.header.stamp}")
    
    rospy.loginfo("Linear Acceleration (m/s^2):")
    rospy.loginfo(f"  x: {msg.linear_acceleration.x}")
    rospy.loginfo(f"  y: {msg.linear_acceleration.y}")
    rospy.loginfo(f"  z: {msg.linear_acceleration.z}")
    
    rospy.loginfo("Angular Velocity (rad/s):")
    rospy.loginfo(f"  x: {msg.angular_velocity.x}")
    rospy.loginfo(f"  y: {msg.angular_velocity.y}")
    rospy.loginfo(f"  z: {msg.angular_velocity.z}")

def imu_subscriber():
    """
    Initializes the IMU subscriber node.
    """
    rospy.init_node('imu_subscriber', anonymous=True)
    rospy.Subscriber("imu_data", Imu, imu_callback)
    rospy.loginfo("IMU Subscriber Node Started. Listening to 'imu_data' topic...")
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_subscriber()
    except rospy.ROSInterruptException:
        pass

