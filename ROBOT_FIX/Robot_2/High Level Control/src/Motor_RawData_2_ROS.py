#!/usr/bin/python

from cmath import inf
import random
from re import I
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from robot_riset.msg import EncoderMotor

rpm1, rpm2, rpm3, rpm4 = 0,0,0,0
pwm1, pwm2, pwm3, pwm4 = 0,0,0,0


def getRPMDataRaw(data):
    global rpm1, rpm2, rpm3, rpm4
    RPM = data

    rpm1, rpm2, rpm3, rpm4 = RPM.x, RPM.y, RPM.z, RPM.w
  

def getPWMDataRaw(data):
    global pwm1, pwm2, pwm3, pwm4
    PWM = data

    pwm1, pwm2, pwm3, pwm4 = PWM.x, PWM.y, PWM.z, PWM.w

def getQuaternionDataRaw(data):
    global q0, q1, q2, q3
    Quat = data
    
    q0 = Quat.x
    q1 = Quat.y
    q2 = Quat.z
    q3 = Quat.w
  



if __name__ == "__main__" :
    try:
        

        rospy.init_node('motor_raw2ros', anonymous=True)
        ENC_pub = rospy.Publisher('/robot_riset/Motor/Feedback', EncoderMotor, queue_size=5)
        rate = rospy.Rate(30)
        rospy.Subscriber("/robot_riset/Motor/Feedback/Raw/rpm", Quaternion, getRPMDataRaw) 
        rospy.Subscriber("/robot_riset/Motor/Feedback/Raw/pwm", Quaternion, getPWMDataRaw) 

        while not rospy.is_shutdown():
            # rospy.loginfo("Hallo")

            enc_msg = EncoderMotor()
            enc_msg.rpm = [Quaternion(rpm1, rpm2, rpm3, rpm4 )]
            enc_msg.pwm = [Quaternion(pwm1, pwm2, pwm3, pwm4 )]

            ENC_pub.publish(enc_msg)
          
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


