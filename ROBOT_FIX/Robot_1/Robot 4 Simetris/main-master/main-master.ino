#include "SerialTransfer.h"
SerialTransfer keSlave;

/// ROS SETUP
#include <ros.h>
#include <robot_riset/TombolMenu.h>
#include <std_msgs/String.h>
#include <robot_riset/MotorEncoder.h>

#include <ESP32Encoder.h>
#include "ESP32TimerInterrupt.h"

#include "PID_Kontrol.h"
#include "Motor.h"
#include "FireTimer.h"


// MODE ROBOT
#define DEBUG_ROS_PWM "debug_ros_pwm"
#define DEBUG_ROS_RPM "debug_ros_rpm"
#define DEBUG_ROBOT "debug_robot"
#define ROBOT_RUN "robot_run"
#define ROBOT_STOP "robot_stop"

String MODE_ROBOT_NOW = ROBOT_STOP;

FireTimer msTimer;
//FireTimer timerNgeprint;

Motor motor3;
Motor motor4;

PID_Kontrol motor3_PID(3, 0, 0, 200, 0, 248, 0, 1023);
PID_Kontrol motor4_PID(1, 0, 0, 200, 0, 248, 0, 1023);

ESP32Timer TimerPID3(0);
ESP32Timer TimerPID4(1);

ESP32Timer ITimer3(2);
ESP32Timer ITimer4(3);

ESP32Encoder encoder3;
ESP32Encoder encoder4;

// Motor 1 - 2 Slave
volatile int rpm1 = 0, rpm2 = 0;

//Motor 3
volatile int lastCount3 = 0;
volatile int deltaCount3 = 0;
volatile int rpm3 = 0;
volatile int encCnt3;

//Motor 4
volatile int lastCount4 = 0;
volatile int deltaCount4 = 0;
volatile int rpm4 = 0;
volatile int encCnt4;

// Setting PWM properties
const int freq = 15000;
const int resolution = 10;

String read1, read2, read3, read4;
String bacaData2;

int pwmOut3;
int pwmOut4;

int enablePID;
int arahPutarMotor3;
int arahPutarMotor4;

/// ---- DEBUG MOTOR ROS ---- //
int debug_PWM_Motor[4] = {0, 0, 0, 0};
//int debug_PWM_Motor_L[4] = {0, 0, 0, 0};
int debug_Arah_Motor[4] = {0, 0, 0, 0};
float debug_RPM_Motor[4] = {0, 0, 0, 0};

//
void IRAM_ATTR TIMER_PID_M3(void) {
  pwmOut3 =  motor3_PID.kalkulasi(abs(rpm3));
  if (pwmOut3 < 0) pwmOut3 = 0;
  if (pwmOut3 > 1023) pwmOut3 = 1023;
}
void IRAM_ATTR TIMER_PID_M4(void) {
  pwmOut4 =  motor4_PID.kalkulasi(abs(rpm4));
  if (pwmOut4 < 0) pwmOut4 = 0;
  if (pwmOut4 > 1023) pwmOut4 = 1023;
}
void IRAM_ATTR TimerHandler3(void) {
  encCnt3 = encoder3.getCount();
  deltaCount3 = encCnt3 - lastCount3;
  rpm3 = ((deltaCount3) * 600) / 745;
  lastCount3 = encCnt3;
}
//
void IRAM_ATTR TimerHandler4(void) {
  encCnt4 = encoder4.getCount();
  deltaCount4 = encCnt4 - lastCount4;
  rpm4 = ((deltaCount4) * 600) / 745;
  lastCount4 = encCnt4;
}


// == == == = ROS Konfigurasi == ==
// Ros Node
ros::NodeHandle Robot;

// ROS MSG
robot_riset::MotorEncoder Motor_1;
robot_riset::MotorEncoder Motor_2;
robot_riset::MotorEncoder Motor_3;
robot_riset::MotorEncoder Motor_4;

// ROS PUB
ros::Publisher Motor1_Pub("robot_riset/Motor_Feedback/Motor1", &Motor_1);
ros::Publisher Motor2_Pub("robot_riset/Motor_Feedback/Motor2", &Motor_2);
ros::Publisher Motor3_Pub("robot_riset/Motor_Feedback/Motor3", &Motor_3);
ros::Publisher Motor4_Pub("robot_riset/Motor_Feedback/Motor4", &Motor_4);

//== ROS SUBS
// Callback di page callback_subs

// Init Subs
ros::Subscriber<std_msgs::String> modeDebug_Sub("robot_riset/Debug", &callback_ModeDebug);
ros::Subscriber<robot_riset::MotorEncoder> Motor1_Sub("robot_riset/Motor_Link/Motor1", &callback_Motor1);
ros::Subscriber<robot_riset::MotorEncoder> Motor2_Sub("robot_riset/Motor_Link/Motor2", &callback_Motor2);
ros::Subscriber<robot_riset::MotorEncoder> Motor3_Sub("robot_riset/Motor_Link/Motor3", &callback_Motor3);
ros::Subscriber<robot_riset::MotorEncoder> Motor4_Sub("robot_riset/Motor_Link/Motor4", &callback_Motor4);


// Serial
void hi()
{
  String read1, read2, read3, read4, read5;

  char arr[200];
  uint16_t recSize = 0;

  recSize = keSlave.rxObj(arr, recSize);
  String Read =  String(arr);
  read1 = parseString(Read, "#", 0);

  if (read1 == "ros_rpm") { // rpm
    read2 = parseString(Read, "#", 1); // rpm 1
    read3 = parseString(Read, "#", 2); // rpm 2

    rpm1 = read2.toInt();
    rpm2 = read3.toInt();
  }
  if (read1 == "ros_pwm") {
    read2 = parseString(Read, "#", 1); // pwm 1
    read3 = parseString(Read, "#", 2); // pwm 2

    debug_PWM_Motor[0] = read2.toInt();
    debug_PWM_Motor[1] = read3.toInt();
  }



}

const functionPtr callbackArr[] = { hi };

void setup() {
  Serial2.begin(57600); // KE Slave pakai baud 57600

  ///////////////////////////////////////////////////////////////// Config Parameters
  configST myConfig;
  myConfig.debug        = true;
  myConfig.callbacks    = callbackArr;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////////////////////////////////////////////////////////////////

  keSlave.begin(Serial2, myConfig);

  Robot.initNode();


  Robot.advertise(Motor1_Pub);
  Robot.advertise(Motor2_Pub);
  Robot.advertise(Motor3_Pub);
  Robot.advertise(Motor4_Pub);

  Robot.subscribe(modeDebug_Sub);
  Robot.subscribe(Motor1_Sub);
  Robot.subscribe(Motor2_Sub);
  Robot.subscribe(Motor3_Sub);
  Robot.subscribe(Motor4_Sub);

  // TIMER BACA ENCODER
  TimerPID3.attachInterruptInterval(50 * 1000, TIMER_PID_M3);
  TimerPID4.attachInterruptInterval(50 * 1000, TIMER_PID_M4);

  ITimer3.attachInterruptInterval(100 * 1000, TimerHandler3);
  ITimer4.attachInterruptInterval(100 * 1000, TimerHandler4);

  ESP32Encoder::useInternalWeakPullResistors = UP;   // Attache pins for use as encoder pins

  msTimer.begin(5000);
  //  timerNgeprint.begin(2000);

  // ---- MOTOR 3
  motor3.pin(32, 33);
  motor3.enc(36, 39);
  motor3.pwm_ch(0, 1);

  // --- MOTOR PID
  motor3_PID.mulai();
  //  motor3_PID.setSetPoints(80);
  motor3_PID.setSampling(50, 50);
  motor3_PID.setKonstanta(1.2, 0, 0);

  // --- MOTOR 3 PWM
  ledcSetup(motor3.pwm_ch1, freq, resolution);
  ledcSetup(motor3.pwm_ch2, freq, resolution);

  ledcAttachPin(motor3.pin1, motor3.pwm_ch1);
  ledcAttachPin(motor3.pin2, motor3.pwm_ch2);
  // -------------------------------------

  // ---- MOTOR 4
  motor4.pin(25, 26);
  motor4.enc(35, 34);
  motor4.pwm_ch(2, 3);

  // --- MOTOR 4 PID
  motor4_PID.mulai();
  motor4_PID.setSetPoints(80);
  motor4_PID.setSampling(0.1, 0.1);
  motor4_PID.setKonstanta(1, 0, 0);

  // --- MOTOR 4 PWM
  ledcSetup(motor4.pwm_ch1, freq, resolution);
  ledcSetup(motor4.pwm_ch2, freq, resolution);

  ledcAttachPin(motor4.pin1, motor4.pwm_ch1);
  ledcAttachPin(motor4.pin2, motor4.pwm_ch2);
  // -------------------------------------

  encoder3.attachHalfQuad(motor3.enc1, motor3.enc2); //encoderA, encoderB
  encoder4.attachHalfQuad(motor4.enc1, motor4.enc2); //encoderA, encoderB

  // count awal - start
  encoder3.setCount(0);
  encoder4.setCount(0);

}

void loop() {
  keSlave.tick();

  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {  // Mode Setting/Debug Robot Melalui ROS MASTER
    //    print_Mode(MODE_ROBOT_NOW);
    if (enablePID == 1) {
      enablePID = 0; // Disable PID Flag
      motor3_PID.reset();
      motor4_PID.reset();
    }

    String printing = "ros_pwm#1#" + String(debug_Arah_Motor[0]) + "#" + String(debug_PWM_Motor[0])
                      + "#2#" + String(debug_Arah_Motor[1]) + "#" + String(debug_PWM_Motor[1]);

    uint16_t sendSize = 0;
    char buf[200];

    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);
    delay(30);

    sendSize = 0;
    buf[200];

    printing = "mode#" + String(DEBUG_ROS_PWM);
    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);

    delay(30);

    // Motor 3
    if (debug_Arah_Motor[2] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor3.pwm_ch1, 0);
      ledcWrite(motor3.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[2] == 1) {
      ledcWrite(motor3.pwm_ch1, debug_PWM_Motor[2]);
      ledcWrite(motor3.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[2] == 2) {
      ledcWrite(motor3.pwm_ch1, 0);
      ledcWrite(motor3.pwm_ch2, debug_PWM_Motor[2]);
    }

    // Motor 4
    if (debug_Arah_Motor[3] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor4.pwm_ch1, 0);
      ledcWrite(motor4.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[3] == 1) {
      ledcWrite(motor4.pwm_ch1, debug_PWM_Motor[3]);
      ledcWrite(motor4.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[3] == 2) {
      ledcWrite(motor4.pwm_ch1, 0);
      ledcWrite(motor4.pwm_ch2, debug_PWM_Motor[3]);
    }

  }
  // RPM - PID
  if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) { //
    if (enablePID == 0) {
      enablePID = 1;
      motor3_PID.mulai();
      motor4_PID.mulai(); ///
    }
    String printing = "ros_rpm#1#" + String(debug_Arah_Motor[0]) + "#" + String(debug_RPM_Motor[0])
                      + "#2#" + String(debug_Arah_Motor[1]) + "#" + String(debug_RPM_Motor[1]);

    uint16_t sendSize = 0;
    char buf[200];

    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);
    delay(30);

    sendSize = 0;
    buf[200];

    printing = "mode#" + String(DEBUG_ROS_RPM);
    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);

    delay(30);
    motor3_PID.setSetPoints(debug_RPM_Motor[2]);
    motor4_PID.setSetPoints(debug_RPM_Motor[3]);


    if (enablePID == 1) {

      //      pwmOut4 = motor4_PID.kalkulasi(abs(rpm4));

      if (debug_Arah_Motor[2] == 0) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[2] == 1) {
        ledcWrite(motor3.pwm_ch1, abs(pwmOut3));
        ledcWrite(motor3.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[2] == 2) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, abs(pwmOut3));
      }

      if (debug_Arah_Motor[3] == 0) {
        ledcWrite(motor4.pwm_ch1, 0);
        ledcWrite(motor4.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[3] == 1) {
        ledcWrite(motor4.pwm_ch1, abs(pwmOut4));
        ledcWrite(motor4.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[3] == 2) {
        ledcWrite(motor4.pwm_ch1, 0);
        ledcWrite(motor4.pwm_ch2, abs(pwmOut4));
      }

      debug_PWM_Motor[2] = abs(pwmOut3);
      debug_PWM_Motor[3] = abs(pwmOut4);
    }

  }


  // Update/Kirim Feedback RPM motor
  Motor_1.rpm = rpm1;
  Motor_1.pwm = debug_PWM_Motor[0];
  Motor_2.rpm = rpm2;
  Motor_2.pwm = debug_PWM_Motor[1];
  Motor_3.rpm = rpm3;
  Motor_3.pwm = debug_PWM_Motor[2];
  Motor_4.rpm = rpm4;
  Motor_4.pwm = debug_PWM_Motor[3];

  Motor1_Pub.publish(&Motor_1);
  Motor2_Pub.publish(&Motor_2);
  Motor3_Pub.publish(&Motor_3);
  Motor4_Pub.publish(&Motor_4);

  Robot.spinOnce();
  delay(1);
}
