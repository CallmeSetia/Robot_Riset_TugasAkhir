#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

class Server:
	def __init__(self):
		print "Test"
	
	def num_callback(self, dat):
		print dat

if __name__ == "__main__":
	rospy.init_node("test_node")
	server = Server()
	try:
		rospy.Subscriber("/test_int", Int32, server.num_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
		
