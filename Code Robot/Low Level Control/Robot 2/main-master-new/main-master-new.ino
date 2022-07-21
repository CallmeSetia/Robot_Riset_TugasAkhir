#include <ros.h>
#include <robot_riset/EncoderMotor.h>
#include <robot_riset/ParamRobot.h>
//#include <robot_riset/ParamPID.h>/

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


#include "SerialTransfer.h"
SerialTransfer keSlave;

#include <ESP32Encoder.h>
#include "ESP32TimerInterrupt.h"
#include "PID_Kontrol.h"
#include "Motor.h"
#include "FireTimer.h"

#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;
float gX, gY, gZ,
      aX, aY, aZ,
      Yaw, Pitch, Roll,
      q0, q1, q2, q3;

int IMU_Time;


#ifdef defined(SAMD)
#define SerialPort SerialUSB
#else
#define SerialPort Serial
#endif
ESP32Timer TimerPID3(0);
ESP32Timer TimerPID4(1);

ESP32Timer ITimer3(2);
ESP32Timer ITimer4(3);

FireTimer msTimer;
FireTimer timerNgeprint1, timerNgeprint2, timerNgeprint3, timerNgeprint4 ;

ESP32Encoder encoder3;
ESP32Encoder encoder4;

Motor motor3;
Motor motor4;

//// Motor 1 - 2 Slave
volatile int rpm1 = 0, pwm1 = 0, pwm2, rpm2 = 0;
//

////Motor 3
volatile int lastCount3 = 0;
volatile int deltaCount3 = 0;
volatile int rpm3 = 0;
volatile int pwm3 = 0;

volatile int encCnt3;

//Motor 4
volatile int lastCount4 = 0;
volatile int deltaCount4 = 0;
volatile int rpm4 = 0;
volatile int pwm4 = 0;

volatile int encCnt4;

//// Setting PWM properties
const int freq = 15000;
const int resolution = 10;

String read1, read2, read3, read4;
String bacaData2;

int pwmOut3;
int pwmOut4;

int enablePID;
int arahPutarMotor3;
int arahPutarMotor4;


PID_Kontrol motor3_PID(3, 0, 0, 200, -248, 248, 0, 1023);
PID_Kontrol motor4_PID(1, 0, 0, 200, -248, 248, 0, 1023);

void IRAM_ATTR TIMER_PID_M3(void) {
  pwmOut3 =  motor3_PID.kalkulasi(rpm3);
}
void IRAM_ATTR TIMER_PID_M4(void) {
  pwmOut4 =  motor4_PID.kalkulasi(rpm4);
}
void IRAM_ATTR TimerHandler3(void) {
  encCnt3 = encoder3.getCount();
  deltaCount3 = encCnt3 - lastCount3;
  rpm3 = ((deltaCount3) * 600) / 728;
  lastCount3 = encCnt3;
}
//
void IRAM_ATTR TimerHandler4(void) {
  encCnt4 = encoder4.getCount();
  deltaCount4 = encCnt4 - lastCount4;
  rpm4 = ((deltaCount4) * 600) / 728;
  lastCount4 = encCnt4;
}

// == == == = ROS Konfigurasi == ==
// Ros Node
ros::NodeHandle Robot;

// ROS MSG
geometry_msgs::Quaternion MotorEnc1;
geometry_msgs::Quaternion MotorEnc2;

geometry_msgs::Quaternion  ParamPWM_Motor;
geometry_msgs::Quaternion  ParamRPM_Motor;

geometry_msgs::Vector3 mpu9250_Accl_ros;
geometry_msgs::Vector3 mpu9250_Gyro_ros;
geometry_msgs::Vector3 mpu9250_Euler_ros;

std_msgs::Int64 mpu9250_IMUTime_ros;
std_msgs::String modeRobot_Msg;

geometry_msgs::Quaternion mpu9250_Quat_ros;


// ROS PUB
ros::Publisher MotorRaw_RPM("robot_riset/Motor/Feedback/Raw/rpm", &MotorEnc1);
ros::Publisher MotorRaw_PWM("robot_riset/Motor/Feedback/Raw/pwm", &MotorEnc2);

ros::Publisher ModeRobot_Pub("robot_riset/ModeRobot", &modeRobot_Msg);

ros::Publisher IMU_Accl_Pub("robot_riset/Imu/DataRaw/Accl", &mpu9250_Accl_ros);
ros::Publisher IMU_Gyro_Pub("robot_riset/Imu/DataRaw/Gyro", &mpu9250_Gyro_ros);
ros::Publisher IMU_Euler_Pub("robot_riset/Imu/DataRaw/Euler", &mpu9250_Euler_ros);
ros::Publisher IMU_Time_Pub("robot_riset/Imu/DataRaw/Time", &mpu9250_IMUTime_ros);
ros::Publisher IMU_Quat_Pub("robot_riset/Imu/DataRaw/Quatternion", &mpu9250_Quat_ros);

ros::Publisher ParamPWM_Motor_Pub("robot_riset/Param/PWM", &ParamPWM_Motor);
ros::Publisher ParamRPM_Motor_Pub("robot_riset/Param/RPM", &ParamRPM_Motor);

//== ROS SUBS
// Init Subs
ros::Subscriber<robot_riset::EncoderMotor> MotorCommand_Sub("robot_riset/Motor/Command", &callback_MotorCommand);


// MODE ROBOT
#define DEBUG_ROS_PWM "debug_ros_pwm"
#define DEBUG_ROS_RPM "debug_ros_rpm"
#define DEBUG_ROBOT_PWM "debug_pwm"
#define DEBUG_ROBOT_RPM "debug_rpm"
#define ROBOT_RUN "robot_run"
#define ROBOT_STOP "robot_stop"

String MODE_ROBOT_NOW = ROBOT_STOP;

#define ROBOT_ID 1


char arr[250];
const functionPtr callbackArr[] = { dariSlave };
char msgbuf[100];
int m1_kp;

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {

  komunikasiSlave_Init();
  ROS_INIT();
  Motor_Init();
  Init_IMU();
  RTOS_Init();
}

void loop() {
  //  loadAllParam();

  controlMotor();
  Read_IMU();
  ROS_Publish();
  
  delay(1);
}

void KomunkasiMasterSlaveTask( void * pvParameters ) {
  for (;;) {

    keSlave.tick();           // Tick untuk Callback dari Slave
    kirimInformasiKeSlave();  // Kirim Ke Slave

    vTaskDelay( 20 / portTICK_PERIOD_MS );
  }
}
