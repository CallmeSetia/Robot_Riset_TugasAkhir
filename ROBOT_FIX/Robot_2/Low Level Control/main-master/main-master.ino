//#include "SerialTransfer.h"
//SerialTransfer keSlave;
//
///// ROS SETUP
//#include <ros.h>
//#include <robot_riset/TombolMenu.h>
//#include <std_msgs/String.h>
//#include <robot_riset/MotorEncoder.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Quaternion.h>
//#include <std_msgs/Int64.h>
//
//#include <SparkFunMPU9250-DMP.h>
//
//MPU9250_DMP imu;
//float gX, gY, gZ,
//      aX, aY, aZ,
//      Yaw, Pitch, Roll,
//      q0, q1, q2, q3;
//
//int IMU_Time;
//
//
//#ifdef defined(SAMD)
//#define SerialPort SerialUSB
//#else
//#define SerialPort Serial
//#endif
//
//#include <ESP32Encoder.h>
//#include "ESP32TimerInterrupt.h"
//
//#include "PID_Kontrol.h"
//#include "Motor.h"
//#include "FireTimer.h"
//
//// MODE ROBOT
//#define DEBUG_ROS_PWM "debug_ros_pwm"
//#define DEBUG_ROS_RPM "debug_ros_rpm"
//#define DEBUG_ROBOT_PWM "debug_robot_pwm"
//#define DEBUG_ROBOT_RPM "debug_robot_rpm"
//#define ROBOT_RUN "robot_run"
//#define ROBOT_STOP "robot_stop"
//
//String MODE_ROBOT_NOW = ROBOT_STOP;
//
//FireTimer msTimer;
////FireTimer timerNgeprint;
//
//Motor motor3;
//Motor motor4;
//
//PID_Kontrol motor3_PID(3, 0, 0, 200, 0, 248, 0, 1023);
//PID_Kontrol motor4_PID(1, 0, 0, 200, 0, 248, 0, 1023);
//
//ESP32Timer TimerPID3(0);
//ESP32Timer TimerPID4(1);
//
//ESP32Timer ITimer3(2);
//ESP32Timer ITimer4(3);
//
//ESP32Encoder encoder3;
//ESP32Encoder encoder4;
//
//// Motor 1 - 2 Slave
//volatile int rpm1 = 0, rpm2 = 0;
//
////Motor 3
//volatile int lastCount3 = 0;
//volatile int deltaCount3 = 0;
//volatile int rpm3 = 0;
//volatile int encCnt3;
//
////Motor 4
//volatile int lastCount4 = 0;
//volatile int deltaCount4 = 0;
//volatile int rpm4 = 0;
//volatile int encCnt4;
//
//// Setting PWM properties
//const int freq = 15000;
//const int resolution = 10;
//
//String read1, read2, read3, read4;
//String bacaData2;
//
//int pwmOut3;
//int pwmOut4;
//
//int enablePID;
//int arahPutarMotor3;
//int arahPutarMotor4;
//
///// ---- DEBUG MOTOR ROS ---- //
//int debug_PWM_Motor[4] = {0, 0, 0, 0};
////int debug_PWM_Motor_L[4] = {0, 0, 0, 0};
//int debug_Arah_Motor[4] = {0, 0, 0, 0};
//float debug_RPM_Motor[4] = {0, 0, 0, 0};
//
////
//void IRAM_ATTR TIMER_PID_M3(void) {
//  pwmOut3 =  motor3_PID.kalkulasi(abs(rpm3));
//  if (pwmOut3 < 0) pwmOut3 = 0;
//  if (pwmOut3 > 1023) pwmOut3 = 1023;
//}
//void IRAM_ATTR TIMER_PID_M4(void) {
//  pwmOut4 =  motor4_PID.kalkulasi(abs(rpm4));
//  if (pwmOut4 < 0) pwmOut4 = 0;
//  if (pwmOut4 > 1023) pwmOut4 = 1023;
//}
//void IRAM_ATTR TimerHandler3(void) {
//  encCnt3 = encoder3.getCount();
//  deltaCount3 = encCnt3 - lastCount3;
//  rpm3 = ((deltaCount3) * 600) / 745;
//  lastCount3 = encCnt3;
//}
////
//void IRAM_ATTR TimerHandler4(void) {
//  encCnt4 = encoder4.getCount();
//  deltaCount4 = encCnt4 - lastCount4;
//  rpm4 = ((deltaCount4) * 600) / 745;
//  lastCount4 = encCnt4;
//}
//
//
//// == == == = ROS Konfigurasi == ==
//// Ros Node
//ros::NodeHandle Robot;
//
//// ROS MSG
//robot_riset::MotorEncoder Motor_1;
//robot_riset::MotorEncoder Motor_2;
//robot_riset::MotorEncoder Motor_3;
//robot_riset::MotorEncoder Motor_4;
//
//
//geometry_msgs::Vector3 mpu9250_Accl_ros;
//geometry_msgs::Vector3 mpu9250_Gyro_ros;
//geometry_msgs::Vector3 mpu9250_Euler_ros;
//
//std_msgs::Int64 mpu9250_IMUTime_ros;
//
//geometry_msgs::Quaternion mpu9250_Quat_ros;
//
//
//
//// ROS PUB
//ros::Publisher Motor1_Pub("robot_riset/Motor_Feedback/Motor1", &Motor_1);
//ros::Publisher Motor2_Pub("robot_riset/Motor_Feedback/Motor2", &Motor_2);
//ros::Publisher Motor3_Pub("robot_riset/Motor_Feedback/Motor3", &Motor_3);
//ros::Publisher Motor4_Pub("robot_riset/Motor_Feedback/Motor4", &Motor_4);
//
//ros::Publisher IMU_Accl_Pub("robot_riset/Imu/DataRaw/Accl", &mpu9250_Accl_ros);
//ros::Publisher IMU_Gyro_Pub("robot_riset/Imu/DataRaw/Gyro", &mpu9250_Gyro_ros);
//ros::Publisher IMU_Euler_Pub("robot_riset/Imu/DataRaw/Euler", &mpu9250_Euler_ros);
//ros::Publisher IMU_Time_Pub("robot_riset/Imu/DataRaw/Time", &mpu9250_IMUTime_ros);
//ros::Publisher IMU_Quat_Pub("robot_riset/Imu/DataRaw/Quatternion", &mpu9250_Quat_ros);
//
//
//
////== ROS SUBS
//// Callback di page callback_subs
//
//// Init Subs
//ros::Subscriber<std_msgs::String> modeDebug_Sub("robot_riset/Debug", &callback_ModeDebug);
//ros::Subscriber<robot_riset::MotorEncoder> Motor1_Sub("robot_riset/Motor_Link/Motor1", &callback_Motor1);
//ros::Subscriber<robot_riset::MotorEncoder> Motor2_Sub("robot_riset/Motor_Link/Motor2", &callback_Motor2);
//ros::Subscriber<robot_riset::MotorEncoder> Motor3_Sub("robot_riset/Motor_Link/Motor3", &callback_Motor3);
//ros::Subscriber<robot_riset::MotorEncoder> Motor4_Sub("robot_riset/Motor_Link/Motor4", &callback_Motor4);
//
//
//// Serial
//void hi()
//{
//  String read1, read2, read3, read4, read5;
//
//  char arr[200];
//  uint16_t recSize = 0;
//
//  recSize = keSlave.rxObj(arr, recSize);
//  String Read =  String(arr);
//  read1 = parseString(Read, "#", 0);
//
//  if (read1 == "ros_rpm") { // rpm
//    read2 = parseString(Read, "#", 1); // rpm 1
//    read3 = parseString(Read, "#", 2); // rpm 2
//
//    rpm1 = read2.toInt();
//    rpm2 = read3.toInt();
//  }
//  if (read1 == "ros_pwm") {
//    read2 = parseString(Read, "#", 1); // pwm 1
//    read3 = parseString(Read, "#", 2); // pwm 2
//
//    debug_PWM_Motor[0] = read2.toInt();
//    debug_PWM_Motor[1] = read3.toInt();
//  }
//  if (read1 == "robot_pwm") {
//    read2 = parseString(Read, "#", 1); // pwm 1
//    read3 = parseString(Read, "#", 2); // pwm 2
//
//    MODE_ROBOT_NOW = DEBUG_ROBOT_PWM;
//    debug_PWM_Motor[2] = read2.toInt();
//    debug_PWM_Motor[3] = read3.toInt();
//  }
//
//
//
//}
//
//const functionPtr callbackArr[] = { hi };
//
//void setup() {
//  Serial2.begin(57600); // KE Slave pakai baud 57600
//
//  ///////////////////////////////////////////////////////////////// Config Parameters
//  configST myConfig;
//  myConfig.debug        = true;
//  myConfig.callbacks    = callbackArr;
//  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
//  /////////////////////////////////////////////////////////////////
//
//  keSlave.begin(Serial2, myConfig);
//
//  Robot.initNode();
//
//
//  Robot.advertise(Motor1_Pub);
//  Robot.advertise(Motor2_Pub);
//  Robot.advertise(Motor3_Pub);
//  Robot.advertise(Motor4_Pub);
//
//
//  Robot.advertise(IMU_Accl_Pub);
//  Robot.advertise(IMU_Gyro_Pub);
//  Robot.advertise(IMU_Quat_Pub);
//  Robot.advertise(IMU_Euler_Pub);
//  Robot.advertise(IMU_Time_Pub);
//
//
//  Robot.subscribe(modeDebug_Sub);
//  Robot.subscribe(Motor1_Sub);
//  Robot.subscribe(Motor2_Sub);
//  Robot.subscribe(Motor3_Sub);
//  Robot.subscribe(Motor4_Sub);
//
//  delay(1000);
//
//  if (imu.begin() != INV_SUCCESS)
//  {
//    while (1)
//    {
//      Robot.loginfo("Unable to communicate with MPU-9250");
//      Robot.loginfo("Check connections, and try again.");
//      delay(3000);
//    }
//  }
//  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//
//  imu.setGyroFSR(2000);
//  imu.setAccelFSR(2);
//  imu.setLPF(5); // Set LPF corner frequency to 5Hz // Can be any of the following: 188, 98, 42, 20, 10, 5
//  imu.setSampleRate(10); // Set sample rate to 10Hz
//
//
//  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
//               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
//               10);
//
//  // TIMER BACA ENCODER
//  TimerPID3.attachInterruptInterval(50 * 1000, TIMER_PID_M3);
//  TimerPID4.attachInterruptInterval(50 * 1000, TIMER_PID_M4);
//
//  ITimer3.attachInterruptInterval(100 * 1000, TimerHandler3);
//  ITimer4.attachInterruptInterval(100 * 1000, TimerHandler4);
//
//  ESP32Encoder::useInternalWeakPullResistors = UP;   // Attache pins for use as encoder pins
//
//  msTimer.begin(5000);
//  //  timerNgeprint.begin(2000);
//
//  // ---- MOTOR 3
//  motor3.pin(32, 33);
//  motor3.enc(36, 39);
//  motor3.pwm_ch(0, 1);
//
//  // --- MOTOR PID
//  motor3_PID.mulai();
//  //  motor3_PID.setSetPoints(80);
//  motor3_PID.setSampling(50, 50);
//  motor3_PID.setKonstanta(1.2, 0, 0);
//
//  // --- MOTOR 3 PWM
//  ledcSetup(motor3.pwm_ch1, freq, resolution);
//  ledcSetup(motor3.pwm_ch2, freq, resolution);
//
//  ledcAttachPin(motor3.pin1, motor3.pwm_ch1);
//  ledcAttachPin(motor3.pin2, motor3.pwm_ch2);
//  // -------------------------------------
//
//  // ---- MOTOR 4
//  motor4.pin(25, 26);
//  motor4.enc(35, 34);
//  motor4.pwm_ch(2, 3);
//
//  // --- MOTOR 4 PID
//  motor4_PID.mulai();
//  motor4_PID.setSetPoints(80);
//  motor4_PID.setSampling(0.1, 0.1);
//  motor4_PID.setKonstanta(1, 0, 0);
//
//  // --- MOTOR 4 PWM
//  ledcSetup(motor4.pwm_ch1, freq, resolution);
//  ledcSetup(motor4.pwm_ch2, freq, resolution);
//
//  ledcAttachPin(motor4.pin1, motor4.pwm_ch1);
//  ledcAttachPin(motor4.pin2, motor4.pwm_ch2);
//  // -------------------------------------
//
//  encoder3.attachHalfQuad(motor3.enc1, motor3.enc2); //encoderA, encoderB
//  encoder4.attachHalfQuad(motor4.enc1, motor4.enc2); //encoderA, encoderB
//
//  // count awal - start
//  encoder3.setCount(0);
//  encoder4.setCount(0);
//
//}
//
//void loop() {
//  keSlave.tick();
//
//  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {  // Mode Setting/Debug Robot Melalui ROS MASTER
//    print_Mode(MODE_ROBOT_NOW);
//    if (enablePID == 1) {
//      enablePID = 0; // Disable PID Flag
//      motor3_PID.reset();
//      motor4_PID.reset();
//    }
//
//    String printing = "ros_pwm#1#" + String(debug_Arah_Motor[0]) + "#" + String(debug_PWM_Motor[0])
//                      + "#2#" + String(debug_Arah_Motor[1]) + "#" + String(debug_PWM_Motor[1]);
//
//    uint16_t sendSize = 0;
//    char buf[200];
//
//    printing.toCharArray(buf, printing.length() + 1);
//
//    sendSize = keSlave.txObj(buf, sendSize);
//    keSlave.sendData(sendSize);
//    delay(30);
//
//    sendSize = 0;
//    buf[200];
//
//    printing = "mode#" + String(DEBUG_ROS_PWM);
//    printing.toCharArray(buf, printing.length() + 1);
//
//    sendSize = keSlave.txObj(buf, sendSize);
//    keSlave.sendData(sendSize);
//
//    delay(30);
//
//    // Motor 3
//    if (debug_Arah_Motor[2] == 0) { // Direction = 0 --> Mati
//      ledcWrite(motor3.pwm_ch1, 0);
//      ledcWrite(motor3.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[2] == 1) {
//      ledcWrite(motor3.pwm_ch1, debug_PWM_Motor[2]);
//      ledcWrite(motor3.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[2] == 2) {
//      ledcWrite(motor3.pwm_ch1, 0);
//      ledcWrite(motor3.pwm_ch2, debug_PWM_Motor[2]);
//    }
//
//    // Motor 4
//    if (debug_Arah_Motor[3] == 0) { // Direction = 0 --> Mati
//      ledcWrite(motor4.pwm_ch1, 0);
//      ledcWrite(motor4.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[3] == 1) {
//      ledcWrite(motor4.pwm_ch1, debug_PWM_Motor[3]);
//      ledcWrite(motor4.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[3] == 2) {
//      ledcWrite(motor4.pwm_ch1, 0);
//      ledcWrite(motor4.pwm_ch2, debug_PWM_Motor[3]);
//    }
//
//  }
//  // RPM - PID
//  if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) { //
//    if (enablePID == 0) {
//      enablePID = 1;
//      motor3_PID.mulai();
//      motor4_PID.mulai(); ///
//    }
//    String printing = "ros_rpm#1#" + String(debug_Arah_Motor[0]) + "#" + String(debug_RPM_Motor[0])
//                      + "#2#" + String(debug_Arah_Motor[1]) + "#" + String(debug_RPM_Motor[1]);
//
//    uint16_t sendSize = 0;
//    char buf[200];
//
//    printing.toCharArray(buf, printing.length() + 1);
//
//    sendSize = keSlave.txObj(buf, sendSize);
//    keSlave.sendData(sendSize);
//    delay(30);
//
//    motor3_PID.setSetPoints(debug_RPM_Motor[2]);
//    motor4_PID.setSetPoints(0);
//
//
//    if (enablePID == 1) {
//
//      //      pwmOut4 = motor4_PID.kalkulasi(abs(rpm4));
//
//      if (debug_Arah_Motor[2] == 0) {
//        ledcWrite(motor3.pwm_ch1, 0);
//        ledcWrite(motor3.pwm_ch2, 0);
//      }
//      if (debug_Arah_Motor[2] == 1) {
//        ledcWrite(motor3.pwm_ch1, abs(pwmOut3));
//        ledcWrite(motor3.pwm_ch2, 0);
//      }
//      if (debug_Arah_Motor[2] == 2) {
//        ledcWrite(motor3.pwm_ch1, 0);
//        ledcWrite(motor3.pwm_ch2, abs(pwmOut3));
//      }
//
//      if (debug_Arah_Motor[3] == 0) {
//        ledcWrite(motor4.pwm_ch1, 0);
//        ledcWrite(motor4.pwm_ch2, 0);
//      }
//      if (debug_Arah_Motor[3] == 1) {
//        ledcWrite(motor4.pwm_ch1, abs(pwmOut4));
//        ledcWrite(motor4.pwm_ch2, 0);
//      }
//      if (debug_Arah_Motor[3] == 2) {
//        ledcWrite(motor4.pwm_ch1, 0);
//        ledcWrite(motor4.pwm_ch2, abs(pwmOut4));
//      }
//
//      debug_PWM_Motor[2] = abs(pwmOut3);
//      debug_PWM_Motor[3] = abs(pwmOut4);
//    }
//
//  }
//  if (MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {
//
//    print_Mode(MODE_ROBOT_NOW);
//    if (enablePID == 1) {
//      enablePID = 0; // Disable PID Flag
//      motor3_PID.reset();
//      motor4_PID.reset();
//    }
//
//    // Motor 3
//    if (debug_Arah_Motor[2] == 0) { // Direction = 0 --> Mati
//      ledcWrite(motor3.pwm_ch1, 0);
//      ledcWrite(motor3.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[2] == 1) {
//      ledcWrite(motor3.pwm_ch1, debug_PWM_Motor[2]);
//      ledcWrite(motor3.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[2] == 2) {
//      ledcWrite(motor3.pwm_ch1, 0);
//      ledcWrite(motor3.pwm_ch2, debug_PWM_Motor[2]);
//    }
//
//    // Motor 4
//    if (debug_Arah_Motor[3] == 0) { // Direction = 0 --> Mati
//      ledcWrite(motor4.pwm_ch1, 0);
//      ledcWrite(motor4.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[3] == 1) {
//      ledcWrite(motor4.pwm_ch1, debug_PWM_Motor[3]);
//      ledcWrite(motor4.pwm_ch2, 0);
//    }
//
//    else if (debug_Arah_Motor[3] == 2) {
//      ledcWrite(motor4.pwm_ch1, 0);
//      ledcWrite(motor4.pwm_ch2, debug_PWM_Motor[3]);
//    }
//
//    delay(30);
//
//    uint16_t sendSize = 0;
//    char buf[200];
//
//    String printing = "feedback_master#" + String(rpm3) + "#" + String(rpm4);
//    printing.toCharArray(buf, printing.length() + 1);
//
//    sendSize = keSlave.txObj(buf, sendSize);
//    keSlave.sendData(sendSize);
//
//  }
//  if ( imu.dataReady() ) {
//    imu.update(UPDATE_ACCEL | UPDATE_GYRO);
//    gX = imu.calcGyro(imu.gx);
//    gY = imu.calcGyro(imu.gy);
//    gZ = imu.calcGyro(imu.gz);
//
//    aX = imu.calcAccel(imu.ax);
//    aY = imu.calcAccel(imu.ay);
//    aZ = imu.calcAccel(imu.az);
//
//  }
//
//  // Check for new data in the FIFO
//  if ( imu.fifoAvailable() )  {
//    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
//    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
//    {
//      // computeEulerAngles can be used -- after updating the
//      // quaternion values -- to estimate roll, pitch, and yaw
//      imu.computeEulerAngles();
//
//      q0 = imu.calcQuat(imu.qw);
//      q1 = imu.calcQuat(imu.qx);
//      q2 = imu.calcQuat(imu.qy);
//      q3 = imu.calcQuat(imu.qz);
//
//      Yaw = imu.yaw;
//      Pitch = imu.pitch;
//      Roll = imu.roll;
//
//      IMU_Time = imu.time;
//      //      Robot.loginfo();
//
//      //      SerialPort.println("Q: " + String(q0, 4) + ", " +
//      //                         String(q1, 4) + ", " + String(q2, 4) +
//      //                         ", " + String(q3, 4));
//      //      SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
//      //                         + String(imu.pitch) + ", " + String(imu.yaw));
//      //      SerialPort.println("Time: " + String(imu.time) + " ms");
//      //      SerialPort.println();
//    }
//  }
//
//
//  mpu9250_Accl_ros.x = aX;
//  mpu9250_Accl_ros.y = aY;
//  mpu9250_Accl_ros.z = aZ;
//
//  mpu9250_Gyro_ros.x = gX;
//  mpu9250_Gyro_ros.y = gY;
//  mpu9250_Gyro_ros.z = gZ;
//
//  mpu9250_Quat_ros.x = q0;
//  mpu9250_Quat_ros.y = q1;
//  mpu9250_Quat_ros.z = q2;
//  mpu9250_Quat_ros.w = q3;
//
//  mpu9250_Euler_ros.x = Yaw;
//  mpu9250_Euler_ros.y = Pitch;
//  mpu9250_Euler_ros.z = Roll;
//
//
//  mpu9250_IMUTime_ros.data = IMU_Time;
//
//
//
//  IMU_Accl_Pub.publish(&mpu9250_Accl_ros);
//  IMU_Gyro_Pub.publish(&mpu9250_Gyro_ros);
//  IMU_Quat_Pub.publish(&mpu9250_Quat_ros);
//  IMU_Euler_Pub.publish(&mpu9250_Euler_ros);
//  IMU_Time_Pub.publish(&mpu9250_IMUTime_ros);
//
//  // Update/Kirim Feedback RPM motor
//  Motor_1.rpm = rpm1;
//  Motor_1.pwm = debug_PWM_Motor[0];
//  Motor_2.rpm = rpm2;
//  Motor_2.pwm = debug_PWM_Motor[1];
//  Motor_3.rpm = rpm3;
//  Motor_3.pwm = debug_PWM_Motor[2];
//  Motor_4.rpm = rpm4;
//  Motor_4.pwm = debug_PWM_Motor[3];
//
//  Motor1_Pub.publish(&Motor_1);
//  Motor2_Pub.publish(&Motor_2);
//  Motor3_Pub.publish(&Motor_3);
//  Motor4_Pub.publish(&Motor_4);
//
//  Robot.spinOnce();
//  delay(1);
//}
