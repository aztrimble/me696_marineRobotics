#!/usr/bin/env python


#import important modules
import rospy
from numpy import arctan, sin, cos, matrix
from sensor_msgs.msg import Imu
from std_msgs.msg import String


#initiate node
#rospy.init_node('IMU_transform_maker', anonymous = True)
#sub = rospy.Subscriber("/razorImu1/data", Imu)
#pub = rospy.Publisher("/razorImu1/calibrated", Imu, queue_size = 10)



#Averag values for Imu1
phi = 0.5916614882
theta = 0.0420611683
mag = 9.215854255



B = matrix([[cos(phi), sin(phi), 0], [-sin(phi), cos(phi), 0], [0,0,1]])
A = matrix([[cos(theta), 0, -sin(theta)], [0,1,0], [sin(theta), 0, cos(theta)]])




#Puts the data through calibration and sends it to a new topic
def imu_callback(msg, pub):
	
	#Takes linear acceleration data and corrects the magnetude
	x = msg.linear_acceleration.x * (9.81/mag)
	y = msg.linear_acceleration.y * (9.81/mag)
	z = msg.linear_acceleration.z * (9.81/mag)

	#Sends the data through the rotation matrices
	raw_accel = matrix([[x],[y],[z]])
	calibrated_accel = A * (B * raw_accel)

	#Takes the whole uncalibrated Imu data and sets to new accelerations
	
	msg.linear_acceleration.x = calibrated_accel.item(0)
	msg.linear_acceleration.y = calibrated_accel.item(1)
	msg.linear_acceleration.z = calibrated_accel.item(2)


	#Publishes the adjusted data to new topic
	pub.publish(msg)


def main():
	rospy.init_node('IMU_calibrator')
	pub = rospy.Publisher("/razorImu1/calibrated", Imu, queue_size = 10)
	rospy.Subscriber('/razorImu1/data', Imu, imu_callback, callback_args=pub)
	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
   
