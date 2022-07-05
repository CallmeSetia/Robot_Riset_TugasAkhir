#!/usr/bin/env python

import rospy
import numpy as np
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped

class Server:
	def __init__(self):
		# spesifikasi robot
		self.alp = [np.radians(45.), np.radians(135.), np.radians(-135.), np.radians(-45.)]
		self.gamma = [np.radians(-45.), np.radians(45.), np.radians(135.), np.radians(-135.)]
		self.l = [0.1305, 0.1305, 0.1305, 0.1305]	#jarak titik tengah robot ke titik tengah roda, satuan dalam meter
		self.r = 0.03		#jari - jari roda, satuan dalam meter
		# variable kinematik
		self.pose = np.matrix([[0.,0.,0.]]).transpose()
		self.pose_des = np.matrix([[0.,0.,0.]]).transpose()
		self.w = np.matrix([[0., 0., 0., 0.]]).transpose()
		self.lamda = np.matrix([[30., 0., 0.], [0., 30., 0.], [0., 0., 30]])
		self.motor_publisher = rospy.Publisher("/robot_riset/Motor/link/rpm", Quaternion, queue_size = 10)
		self.Jr = self.get_jacobianR()
		print("Inverse Kinematics Ready")
	
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
	
	def pose_callback(self, dat):
		self.pose[0,0] = dat.pose.pose.position.x
		self.pose[1,0] = dat.pose.pose.position.y
		orientation_list = [dat.pose.pose.orientation.x, dat.pose.pose.orientation.y, dat.pose.pose.orientation.z, dat.pose.pose.orientation.w]
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
		self.pose[2,0] = yaw
		self.main()
	
	def pose_desired_callback(self, dat):
		self.pose_des[0,0] = dat.x
		self.pose_des[1,0] = dat.y
		self.pose_des[2,0] = np.radians(dat.z)
	
	def main(self):
		J = self.get_jacobianW(self.pose[2,0], self.Jr)
		error = self.pose_des - self.pose
		if(np.linalg.norm(error) < 0.2):
			kp = 0.
		else:
			kp = self.lamda
		self.w = J * kp * error
		motor.x = self.w[0,0]
		motor.y = self.w[1,0]
		motor.z = self.w[2,0]
		motor.w = self.w[3,0]
		self.motor_publisher.publish(motor)
		print self.w

if __name__ == "__main__":
	rospy.init_node("inverse_kinematic_node")
	server = Server()
	motor = Quaternion()
	try:
		rospy.Subscriber("/robot_riset/pose_desired", Vector3, server.pose_desired_callback)
		rospy.Subscriber("/robot_riset/odom", Odometry, server.pose_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
