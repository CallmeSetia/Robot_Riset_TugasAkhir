#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Int32, Int32MultiArray, Empty
import numpy as np
import tf

class Server:
	def __init__(self):
		print "Program Ready"
		# imu's varibles
		self.imu_data = Imu()
		self.last_data = [0., 0., 0.]
		self.orientation = [0., 0., 0.]
		self.Ts = 0.03
		self.imu_ref = 0
		self.final_orientation = 0
		self.roll_dot = 0.
		# robot's specification
		self.alp = [np.radians(45.), np.radians(135.), np.radians(-135.), np.radians(-45.)]
		self.gamma = [np.radians(-45.), np.radians(45.), np.radians(135.), np.radians(-135.)]
		self.l = [0.125, 0.125, 0.125, 0.125]	#jarak titik tengah robot ke titik tengah roda, satuan dalam meter
		self.r = 0.03		#jari - jari roda, satuan dalam meter
		# kinematics variables
		self.w = np.matrix([[0., 0., 0., 0.]]).transpose()
		self.pose = np.matrix([[0., 0., 0.]]).transpose()
		self.pose_dot = np.matrix([[0., 0., 0.]]).transpose()
		self.scale = [1., 1., 1.] # offset sensor [x, y, yaw]
		self.current_time = rospy.Time.now()
		self.Jr = self.get_jacobianR()
		self.dt = 0.1
		# ROS Publisher node
		self.odometry_publisher = rospy.Publisher("/robot_riset/odom", Odometry, queue_size = 50)
	
	def get_jacobianR(self):
		j = np.matrix([[0., 0., 0.], [0.,0.,0.], [0.,0.,0.], [0.,0.,0.]])
		j[0,0] = np.cos(self.alp[0])
		j[0,1] = np.sin(self.alp[0])
		j[0,2] = (self.l[0]*np.cos(self.gamma[0])*np.sin(self.alp[0])) - (self.l[0]*np.sin(self.gamma[0])*np.cos(self.alp[0]))
		j[1,0] = np.cos(self.alp[1])
		j[1,1] = np.sin(self.alp[1])
		j[1,2] = (self.l[1]*np.cos(self.gamma[1])*np.sin(self.alp[1])) - (self.l[1]*np.sin(self.gamma[1])*np.cos(self.alp[1]))
		j[2,0] = np.cos(self.alp[2])
		j[2,1] = np.sin(self.alp[2])
		j[2,2] = (self.l[2]*np.cos(self.gamma[2])*np.sin(self.alp[2])) - (self.l[2]*np.sin(self.gamma[2])*np.cos(self.alp[2]))
		j[3,0] = np.cos(self.alp[3])
		j[3,1] = np.sin(self.alp[3])
		j[3,2] = (self.l[3]*np.cos(self.gamma[3])*np.sin(self.alp[3])) - (self.l[3]*np.sin(self.gamma[3])*np.cos(self.alp[3]))		
		return self.r*j
	
	def get_jacobianW(self, th, Jr):
		rotZ = np.matrix([[np.cos(th), np.sin(th), 0.], [-np.sin(th), np.cos(th), 0.], [0., 0., 1.]])
		J = Jr * rotZ
		return J
	
	def imu_callback(self, dat): # subscribe imu sensor euler data
		self.imu_data = dat
		orientation_list = [dat.orientation.x, dat.orientation.y, dat.orientation.z, dat.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
		self.roll_dot = (roll - self.last_data[0]) / self.Ts # menghitung kecepatan perubahan arah hadap
		#pitch_dot = (pitch - self.last_data[1]) / self.Ts
		#yaw_dot = (yaw - self.last_data[2]) / self.Ts
		self.orientation[0] = self.orientation[0] + self.roll_dot * self.Ts # menghitung arah hadap baru bergantung dari kecepatan perubahan arah hadap
		#self.orientation[1] = self.orientation[1] + pitch_dot * self.Ts
		#self.orientation[2] = self.orientation[2] + yaw_dot * self.Ts
		self.final_orientation = self.orientation[0] - self.imu_ref # arah hadap berdasarkan nilai referensi
		if(self.final_orientation > np.radians(180.)):
			self.final_orientation = self.final_orientation - np.radians(360.)
		elif(self.final_orientation < np.radians(-180.)):
			self.final_orientation = np.radians(360.) - self.final_orientation
		self.last_data[0] = roll
		#self.last_data[1] = pitch
		#self.last_data[2] = yaw
		#print np.degrees(self.orientation[0]), np.degrees(self.imu_ref), np.degrees(self.final_orientation)
	
	def rpm_callback(self, dat):
		self.w[0,0] = dat.x
		self.w[1,0] = dat.y
		self.w[2,0] = dat.z
		self.w[3,0] = dat.w
		self.compute_odometry()
	
	def compute_odometry(self):
		#time stamped
		self.current_time = rospy.Time.now()		
		#==========================
		J = self.get_jacobianW(self.pose[2,0], self.Jr)
		Ji = np.linalg.pinv(J)
		pose_dot = Ji * self.w				
		pose_dot[0,0] = pose_dot[0,0] * self.scale[0]
		pose_dot[1,0] = pose_dot[1,0] * self.scale[1]
		#pose_dot[2,0] = pose_dot[2,0] * self.scale[2]
		self.pose[0,0] = self.pose[0,0] + pose_dot[0,0] * self.dt#
		self.pose[1,0] = self.pose[1,0] + pose_dot[1,0] * self.dt#
		self.pose[2,0] = self.final_orientation
		pose_dot[2,0] = self.roll_dot
		self.imu_dot = 0		
		#======================= standard odometry ros =============================
		vx = pose_dot[0,0]
		vy = pose_dot[1,0]
		vth = pose_dot[2,0]		
		x = self.pose[0,0]
		y = self.pose[1,0]
		orientation = self.pose[2,0]
		#quartenion
		self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, orientation)
		#print(self.odom_quat)
		odom_broadcaster.sendTransform(
			(x, y, 0.),
			self.odom_quat,
			self.current_time,
			"base_footprint",
			"odom"
		)
		odom.header.stamp = self.current_time
		odom.header.frame_id = "odom"
		odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*self.odom_quat))
		#odom.pose.covariance[0] = 0.0001
		#odom.pose.covariance[7] = 0.0001
		#odom.pose.covariance[35] = 0.0001
		odom.child_frame_id = "base_footprint"
		odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
		#odom.twist.covariance[0] = 0.00001
		#odom.twist.covariance[7] = 0.00001
		#odom.twist.covariance[35] = 0.00001
		#=======================================================================
		print self.pose
		self.odometry_publisher.publish(odom)
	
	def reference_update(self, dat):
		self.imu_ref = self.orientation[0]
		print "reference Updated"

if __name__ == "__main__":
	rospy.init_node("robot_imu_node")
	server = Server()
	odom = Odometry()
	odom_broadcaster = tf.TransformBroadcaster()
	try:
		rospy.Subscriber("/robot_riset/Imu", Imu, server.imu_callback)
		rospy.Subscriber("/set_imu_ref", Int32, server.reference_update)
		rospy.Subscriber("/robot_riset/Motor", Quaternion, server.rpm_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
		
