#!/usr/bin/env python


#import important modules
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String


imu = Imu()

imu.header.frame_id = "base_link"

imu.orientation = Quaternion(0, 0, 0, 1)
imu.angular_velocity = Vector3(0, 0, 0)






def main():

	rospy.init_node('IMU_fakeline')
	pub = rospy.Publisher("/razorImu1/calibrated", Imu, queue_size = 10)
	pub2 = rospy.Publisher("/razorImu2/calibrated", Imu, queue_size = 10)

	start_time = rospy.Time.now()
	t = rospy.Duration(0)
	s = 8
#wait at start
	while t < (rospy.Time(s)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		t2 = t
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)

		
#first leg
	while t < (rospy.Time(s+2)-rospy.Time(0)) and t > (rospy.Time(s)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 1
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+12)-rospy.Time(0)) and t > (rospy.Time(s+2)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+14)-rospy.Time(0)) and t > (rospy.Time(s+12)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = -1
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
 	
 	
#second leg
	s = s+14
	while t < (rospy.Time(s+2)-rospy.Time(0)) and t > (rospy.Time(s)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = -1
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+12)-rospy.Time(0)) and t > (rospy.Time(s+2)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+14)-rospy.Time(0)) and t > (rospy.Time(s+12)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 1
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
 	
 	
#third leg
	s = s+14
	while t < (rospy.Time(s+2)-rospy.Time(0)) and t > (rospy.Time(s)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = -1
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+12)-rospy.Time(0)) and t > (rospy.Time(s+2)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+14)-rospy.Time(0)) and t > (rospy.Time(s+12)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 1
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
 	
 	
#fourth leg
	s = s+14
	while t < (rospy.Time(s+2)-rospy.Time(0)) and t > (rospy.Time(s)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 1
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+12)-rospy.Time(0)) and t > (rospy.Time(s+2)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = 0
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)
	while t < (rospy.Time(s+14)-rospy.Time(0)) and t > (rospy.Time(s+12)-rospy.Time(0)):
		t =rospy.Time.now() - start_time
		imu.linear_acceleration.x = 0
		imu.linear_acceleration.y = -1
		imu.linear_acceleration.z = 9.81
		imu.header.stamp = rospy.Time.now()
		pub.publish(imu)
		pub2.publish(imu)

	imu.linear_acceleration.x = 0
	imu.linear_acceleration.y = 0
	imu.linear_acceleration.z = 9.81
	imu.header.stamp = rospy.Time.now()
	pub.publish(imu)
	pub2.publish(imu)
 	
 	

	


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
   
