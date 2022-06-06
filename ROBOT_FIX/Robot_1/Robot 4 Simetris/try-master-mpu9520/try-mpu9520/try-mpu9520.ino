
/// ROS SETUP
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int64.h>

#include <SparkFunMPU9250-DMP.h>

#ifdef defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif

MPU9250_DMP imu;
float gX, gY, gZ,
      aX, aY, aZ,
      Yaw, Pitch, Roll,
      q0, q1, q2, q3;

int IMU_Time;

//== == == = ROS Konfigurasi == ==
// Ros Node
ros::NodeHandle Robot;

// ROS MSG
geometry_msgs::Vector3 mpu9250_Accl_ros;
geometry_msgs::Vector3 mpu9250_Gyro_ros;
geometry_msgs::Vector3 mpu9250_Euler_ros;

std_msgs::Int64 mpu9250_IMUTime_ros;

geometry_msgs::Quaternion mpu9250_Quat_ros;


// ROS PUB
ros::Publisher IMU_Accl_Pub("robot_riset/Imu/DataRaw/Accl", &mpu9250_Accl_ros);
ros::Publisher IMU_Gyro_Pub("robot_riset/Imu/DataRaw/Gyro", &mpu9250_Gyro_ros);
ros::Publisher IMU_Euler_Pub("robot_riset/Imu/DataRaw/Euler", &mpu9250_Euler_ros);
ros::Publisher IMU_Time_Pub("robot_riset/Imu/DataRaw/Time", &mpu9250_IMUTime_ros);
ros::Publisher IMU_Quat_Pub("robot_riset/Imu/DataRaw/Quatternion", &mpu9250_Quat_ros);


void setup() {
  Robot.initNode();
  Robot.advertise(IMU_Accl_Pub);
  Robot.advertise(IMU_Gyro_Pub);
  Robot.advertise(IMU_Quat_Pub);
  Robot.advertise(IMU_Euler_Pub);
  Robot.advertise(IMU_Time_Pub);

  delay(1000);

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Robot.loginfo("Unable to communicate with MPU-9250");
      Robot.loginfo("Check connections, and try again.");
      delay(3000);
    }
  }


  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(5); // Set LPF corner frequency to 5Hz // Can be any of the following: 188, 98, 42, 20, 10, 5
  imu.setSampleRate(10); // Set sample rate to 10Hz


  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void loop() {
  if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO);
    gX = imu.calcGyro(imu.gx);
    gY = imu.calcGyro(imu.gy);
    gZ = imu.calcGyro(imu.gz);

    aX = imu.calcAccel(imu.ax);
    aY = imu.calcAccel(imu.ay);
    aZ = imu.calcAccel(imu.az);

  }

  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();

      q0 = imu.calcQuat(imu.qw);
      q1 = imu.calcQuat(imu.qx);
      q2 = imu.calcQuat(imu.qy);
      q3 = imu.calcQuat(imu.qz);

      Yaw = imu.yaw;
      Pitch = imu.pitch;
      Roll = imu.roll;

      IMU_Time = imu.time;
      //      Robot.loginfo();

      //      SerialPort.println("Q: " + String(q0, 4) + ", " +
      //                         String(q1, 4) + ", " + String(q2, 4) +
      //                         ", " + String(q3, 4));
      //      SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
      //                         + String(imu.pitch) + ", " + String(imu.yaw));
      //      SerialPort.println("Time: " + String(imu.time) + " ms");
      //      SerialPort.println();
    }
  }


  mpu9250_Accl_ros.x = aX;
  mpu9250_Accl_ros.y = aY;
  mpu9250_Accl_ros.z = aZ;

  mpu9250_Gyro_ros.x = gX;
  mpu9250_Gyro_ros.y = gY;
  mpu9250_Gyro_ros.z = gZ;

  mpu9250_Quat_ros.x = q0;
  mpu9250_Quat_ros.y = q1;
  mpu9250_Quat_ros.z = q2;
  mpu9250_Quat_ros.w = q3;

  mpu9250_Euler_ros.x = Yaw;
  mpu9250_Euler_ros.y = Pitch;
  mpu9250_Euler_ros.z = Roll;


  mpu9250_IMUTime_ros.data = IMU_Time;



  IMU_Accl_Pub.publish(&mpu9250_Accl_ros);
  IMU_Gyro_Pub.publish(&mpu9250_Gyro_ros);
  IMU_Quat_Pub.publish(&mpu9250_Quat_ros);
  IMU_Euler_Pub.publish(&mpu9250_Euler_ros);
  IMU_Time_Pub.publish(&mpu9250_IMUTime_ros);

  Robot.spinOnce();
  delay(1);

}
