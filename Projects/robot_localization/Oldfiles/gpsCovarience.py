#!/usr/bin/env python


#import important modules
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String







#Puts the data through calibration and sends it to a new topic
def imu_callback(msg, pub):
	
	#Takes linear acceleration data and corrects the magnetude
	msg.position_covariance = [1, 0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0, 1]


	#Takes the whole uncalibrated Imu data and sets to new accelerations
	

	#Publishes the adjusted data to new topic
	pub.publish(msg)


def main():
	rospy.init_node('IMU_calibrator')
	pub = rospy.Publisher("/kanaloa/android/gps/covariance", NavSatFix, queue_size = 10)
	rospy.Subscriber('/kanaloa/android/gps', NavSatFix, imu_callback, callback_args=pub)
	rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
   
