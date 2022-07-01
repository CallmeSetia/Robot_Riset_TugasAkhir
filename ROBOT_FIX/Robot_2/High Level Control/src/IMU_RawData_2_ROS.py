#!/usr/bin/python

from cmath import inf
import random
from re import I
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from robot_riset.msg import EncoderMotor


accl_X, accl_Y, accl_Z = 0,0,0
gyro_X, gyro_Y, gyro_Z = 0,0,0
q0, q1, q2, q3 = 0,0,0,0

def getAcclDataRaw(data):
    global accl_X, accl_Y, accl_Z
    Accl = data

    accl_X = Accl.x
    accl_Y = Accl.y
    accl_Z = Accl.z


def getGyroDataRaw(data):
    global gyro_X, gyro_Y, gyro_Z
    Gyro = data

    gyro_X = Gyro.x
    gyro_Y = Gyro.y
    gyro_Z = Gyro.z

def getQuaternionDataRaw(data):
    global q0, q1, q2, q3
    Quat = data
    
    q0 = Quat.x
    q1 = Quat.y
    q2 = Quat.z
    q3 = Quat.w
  



if __name__ == "__main__" :
    try:
        

        rospy.init_node('imu_raw2ros', anonymous=True)
        IMU_pub = rospy.Publisher('/robot_riset/Imu/', Imu, queue_size=5)
        ENC_pub = rospy.Publisher('/robot_riset/Motor/Feedback', EncoderMotor, queue_size=5)
        
        rospy.Subscriber("/robot_riset/Imu/DataRaw/Accl", Vector3, getAcclDataRaw) 
        rospy.Subscriber("/robot_riset/Imu/DataRaw/Gyro", Vector3, getGyroDataRaw) 
        rospy.Subscriber("/robot_riset/Imu/DataRaw/Quatternion", Quaternion, getQuaternionDataRaw)
        rate = rospy.Rate(30)

        SENSORS_DPS_TO_RADS = 0.017453293
        SENSORS_GRAVITY_EARTH = 9.80665

        while not rospy.is_shutdown():
            # rospy.loginfo("Hallo")

            imu_Msg = Imu()
            imu_Msg.header.stamp = rospy.Time.now();
            imu_Msg.header.frame_id = "imu";
            
            imu_Msg.orientation.x = q0 
            imu_Msg.orientation.y = q1
            imu_Msg.orientation.z = q2
            imu_Msg.orientation.w = q3

            imu_Msg.angular_velocity.x = gyro_X  * SENSORS_DPS_TO_RADS;
            imu_Msg.angular_velocity.y = gyro_Y  * SENSORS_DPS_TO_RADS;
            imu_Msg.angular_velocity.z = gyro_Z  * SENSORS_DPS_TO_RADS;

            imu_Msg.linear_acceleration.x = accl_X * SENSORS_GRAVITY_EARTH;
            imu_Msg.linear_acceleration.y = accl_Y * SENSORS_GRAVITY_EARTH;
            imu_Msg.linear_acceleration.z = accl_Z * SENSORS_GRAVITY_EARTH;
            
            IMU_pub.publish(imu_Msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


