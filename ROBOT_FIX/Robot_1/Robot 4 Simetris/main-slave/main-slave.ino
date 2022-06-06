
#include "SerialTransfer.h"

#include <ESP32Encoder.h>
#include "ESP32TimerInterrupt.h"

#include "PID_Kontrol.h"
#include "Motor.h"
#include "FireTimer.h"

SerialTransfer keMaster;
FireTimer msTimer;
//FireTimer timerNgeprint;

Motor motor1;
Motor motor2;

PID_Kontrol motor1_PID(1, 0, 0, 80, -248, 248, 0, 1023);
PID_Kontrol motor2_PID(1, 0, 0, 80, -248, 248, 0, 1023);

ESP32Timer ITimer1(0);
ESP32Timer ITimer2(1);
ESP32Timer TimerPID1(2);
ESP32Timer TimerPID2(3);

ESP32Encoder encoder1;
ESP32Encoder encoder2;

//Motor 1
volatile int lastCount1 = 0;
volatile int deltaCount1 = 0;
volatile int rpm1 = 0;
volatile int encCnt1;

//Motor 2
volatile int lastCount2 = 0;
volatile int deltaCount2 = 0;
volatile int rpm2 = 0;
volatile int encCnt2;

// Setting PWM properties
const int freq = 15000;
const int resolution = 10;

String read1, read2, read3, read4;
String bacaData2;

int pwmOut1;
int pwmOut2;

int enablePID;
int arahPutarMotor1;
int arahPutarMotor2;

void IRAM_ATTR TIMER_PID_M1(void) {
  pwmOut1 =  motor1_PID.kalkulasi(abs(rpm1));
  if (pwmOut1 < 0) pwmOut1 = 0;
  if (pwmOut1 > 1023) pwmOut1 = 1023;
}
void IRAM_ATTR TIMER_PID_M2(void) {
  pwmOut2 =  motor2_PID.kalkulasi(abs(rpm2));
  if (pwmOut2 < 0) pwmOut2 = 0;
  if (pwmOut2 > 1023) pwmOut2 = 1023;
}


//
void IRAM_ATTR TimerHandler1(void) {
  encCnt1 = encoder1.getCount();
  deltaCount1 = encCnt1 - lastCount1;
  rpm1 = ((deltaCount1) * 600) / 745;
  lastCount1 = encCnt1;
}
//
void IRAM_ATTR TimerHandler2(void) {
  encCnt2 = encoder2.getCount();
  deltaCount2 = encCnt2 - lastCount2;
  rpm2 = ((deltaCount2) * 600) / 745;
  lastCount2 = encCnt2;
}


// MODE ROBOT
#define DEBUG_ROS_PWM "debug_ros_pwm"
#define DEBUG_ROS_RPM "debug_ros_rpm"
#define DEBUG_ROBOT_PWM "debug_robot_pwm"
#define DEBUG_ROBOT_RPM "debug_robot_rpm"
#define ROBOT_RUN "robot_run"
#define ROBOT_STOP "robot_stop"

String MODE_ROBOT_NOW = ROBOT_STOP;

#define ROBOT_ID 1

/// ---- DEBUG MOTOR ROS ---- //
int debug_PWM_Motor[4] = {0, 0, 0, 0};
//int debug_PWM_Motor_L[4] = {0, 0, 0, 0};
int debug_Arah_Motor[4] = {0, 0, 0, 0};
float debug_RPM_Motor[4] = {0, 0, 0, 0};


int TombolMenu[4] = {15, 4, 18, 19};

const char* TampilanAtas[][20] = {
  {"Main Menu Robot:", "Starting Robot", "PWM MTR", "PID TUNING:"}
};

const char* TampilanBawah[][20] = {
  {"> START", "> INFO PARAM", "> TES PWM MOTOR", "> TUNING PID"},
};


//Serial Komunikasi

// -- FUNCTION BIASA
String parseString(String data, char separator[], int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator[0] || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void dari_master()
{

  String read1, read2, read3, read4, read5, read6, read7;

  char arr[200];
  uint16_t recSize = 0;

  recSize = keMaster.rxObj(arr, recSize);
  String Read = String(arr);


  read1 = parseString(Read, "#", 0);

  if (read1 == "mode") { // mode#debug_ros_pwm
    read2 = parseString(Read, "#", 1);

    if (read2 == DEBUG_ROS_PWM) {
//      Serial.println("DEBUG PWM");
      MODE_ROBOT_NOW = DEBUG_ROS_PWM;
    }
    else if (read2 == DEBUG_ROS_RPM) {
//      Serial.println("DEBUG RPM");
      MODE_ROBOT_NOW = DEBUG_ROS_RPM;
    }
    else if (read2 == DEBUG_ROBOT_PWM) {
      MODE_ROBOT_NOW = DEBUG_ROBOT_PWM;
    }
    else if (read2 == DEBUG_ROBOT_RPM) {
      MODE_ROBOT_NOW = DEBUG_ROBOT_RPM;
    }
    else if (read2 == ROBOT_RUN) {
      MODE_ROBOT_NOW = ROBOT_RUN;
    }
    else if (read2 == ROBOT_STOP) {
      MODE_ROBOT_NOW = ROBOT_STOP;
    }
  }

  if (read1 == "ros_pwm") { // ros_pwm#1#2#200#2#1#300
    if ( MODE_ROBOT_NOW == DEBUG_ROS_PWM ) {
      //      Serial.println("Dari Master" + String(Read));
      read2 = parseString(Read, "#", 1); // ID Motor
      read3 = parseString(Read, "#", 2); // Direction Motor 1
      read4 = parseString(Read, "#", 3); // PWM u/ Motor 1
      read5 = parseString(Read, "#", 4); // ID Motor
      read6 = parseString(Read, "#", 5); // Direction Motor 2
      read7 = parseString(Read, "#", 6); // PWM u/ Motor 2

      int dirMotor1 = read3.toInt(),
          dirMotor2 = read6.toInt(),
          pwmMotor1 = read4.toInt(),
          pwmMotor2 = read7.toInt();

      debug_PWM_Motor[0] = pwmMotor1;
      debug_PWM_Motor[1] = pwmMotor2;

      debug_Arah_Motor[0] = dirMotor1;
      debug_Arah_Motor[1] = dirMotor2;
    }
  }

  if (read1 == "ros_rpm") { // ros_rpm#1#2#200#2#1#300
    if ( MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
      //      Serial.println("Dari Master" + String(Read));
      read2 = parseString(Read, "#", 1); // ID Motor
      read3 = parseString(Read, "#", 2); // Direction Motor 1
      read4 = parseString(Read, "#", 3); // RPM u/ Motor 1
      read5 = parseString(Read, "#", 4); // ID Motor
      read6 = parseString(Read, "#", 5); // Direction Motor 2
      read7 = parseString(Read, "#", 6); // RPM u/ Motor 2

      int dirMotor1 = read3.toInt(),
          dirMotor2 = read6.toInt(),
          rpmMotor1 = read4.toInt(),
          rpmMotor2 = read7.toInt();

      debug_RPM_Motor[0] = rpmMotor1;
      debug_RPM_Motor[1] = rpmMotor2;

      debug_Arah_Motor[0] = dirMotor1;
      debug_Arah_Motor[1] = dirMotor2;
    }
  }
  delay(5);
}
const functionPtr callbackArr[] = { dari_master };


void kirimFeedbackRPMMotor() {

  delay(10);
  uint16_t sendSize = 0;
  char buf[200];

  String printing = "ros_rpm#" + String(rpm1) + "#" + String(rpm2);
  printing.toCharArray(buf, printing.length() + 1);
//  Serial.println(buf);
  sendSize = keMaster.txObj(buf, sendSize);
  keMaster.sendData(sendSize);
}


void kirimFeedbackPWMMotor() {

  delay(10);
  uint16_t sendSize = 0;
  char buf[200];

  String printing = "ros_rpm#" + String(debug_PWM_Motor[0]) + "#" + String(debug_PWM_Motor[1]);
  printing.toCharArray(buf, printing.length() + 1);
  //  Serial.println(buf);
  sendSize = keMaster.txObj(buf, sendSize);
  keMaster.sendData(sendSize);
}

void setup() {

  Serial.begin(9600);
  Serial2.begin(57600);
  ///////////////////////////////////////////////////////////////// Config Parameters
  configST myConfig;
  myConfig.debug        = true;
  myConfig.callbacks    = callbackArr;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////////////////////////////////////////////////////////////////
  keMaster.begin(Serial2, myConfig);


  // TIMER BACA ENCODER

  TimerPID1.attachInterruptInterval(50 * 1000, TIMER_PID_M1);
  TimerPID2.attachInterruptInterval(50 * 1000, TIMER_PID_M2);
  ITimer1.attachInterruptInterval(100 * 1000, TimerHandler1);
  ITimer2.attachInterruptInterval(100 * 1000, TimerHandler2);

  ESP32Encoder::useInternalWeakPullResistors = UP;   // Attache pins for use as encoder pins

  msTimer.begin(5000);
  //  timerNgeprint.begin(2000);

  // ---- MOTOR 1
  motor2.pin(32, 33);
  motor2.enc(35, 34);
  motor2.pwm_ch(0, 1);

  // --- MOTOR PID
  motor2_PID.mulai();
  motor2_PID.setSetPoints(80);
  motor2_PID.setSampling(0.10, 0.1);
  motor2_PID.setKonstanta(1, 0, 0);

  // --- MOTOR 1 PWM
  ledcSetup(motor2.pwm_ch1, freq, resolution);
  ledcSetup(motor2.pwm_ch2, freq, resolution);

  ledcAttachPin(motor2.pin1, motor2.pwm_ch1);
  ledcAttachPin(motor2.pin2, motor2.pwm_ch2);
  // -------------------------------------

  // ---- MOTOR 2
  motor1.pin(25, 26);
  motor1.enc(36, 39);
  motor1.pwm_ch(2, 3);

  // --- MOTOR 4 PID
  motor1_PID.mulai();
  motor1_PID.setSetPoints(80);
  motor1_PID.setSampling(0.1, 0.1);
  motor1_PID.setKonstanta(1, 0, 0);

  // --- MOTOR 4 PWM
  ledcSetup(motor1.pwm_ch1, freq, resolution);
  ledcSetup(motor1.pwm_ch2, freq, resolution);

  ledcAttachPin(motor1.pin1, motor1.pwm_ch1);
  ledcAttachPin(motor1.pin2, motor1.pwm_ch2);
  // -------------------------------------

  encoder2.attachHalfQuad(motor1.enc1, motor1.enc2); //encoderA, encoderB
  encoder1.attachHalfQuad(motor2.enc1, motor2.enc2); //encoderA, encoderB

  // count awal - start
  encoder1.setCount(0);
  encoder2.setCount(0);


}

void loop() {
  keMaster.tick();
  uint16_t sendSize = 0;
  char buf[200];

  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {  // Mode Setting/Debug Robot Melalui ROS MASTER
    if (enablePID == 1) {
      enablePID = 0; // Disable PID Flag
      motor1_PID.reset();
      motor2_PID.reset();
    }
    // Motor 1
    if (debug_Arah_Motor[0] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor1.pwm_ch1, 0);
      ledcWrite(motor1.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[0] == 1) {
      ledcWrite(motor1.pwm_ch1, debug_PWM_Motor[0]);
      ledcWrite(motor1.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[0] == 2) {
      ledcWrite(motor1.pwm_ch1, 0);
      ledcWrite(motor1.pwm_ch2, debug_PWM_Motor[0]);
    }

    // Motor 2
    if (debug_Arah_Motor[1] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor2.pwm_ch1, 0);
      ledcWrite(motor2.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[1] == 1) {
      ledcWrite(motor2.pwm_ch1, debug_PWM_Motor[1]);
      ledcWrite(motor2.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[1] == 2) {
      ledcWrite(motor2.pwm_ch1, 0);
      ledcWrite(motor2.pwm_ch2, debug_PWM_Motor[1]);
    }

  }
  if (MODE_ROBOT_NOW  == DEBUG_ROS_RPM ) {
    if (enablePID == 0) {
      enablePID = 1;
      motor1_PID.mulai();
      motor2_PID.mulai(); //
    }


    motor1_PID.setSetPoints(debug_RPM_Motor[0]);
    motor2_PID.setSetPoints(debug_RPM_Motor[1]);


    if (enablePID == 1) {

      // Motor 1
      if (debug_Arah_Motor[0] == 0) {
        ledcWrite(motor1.pwm_ch1, 0);
        ledcWrite(motor1.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[0] == 1) {
        ledcWrite(motor1.pwm_ch1, abs(500));
        ledcWrite(motor1.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[0] == 2) {
        ledcWrite(motor1.pwm_ch1, 0);
        ledcWrite(motor1.pwm_ch2, abs(500));
      }

      // Motor 2

      if (debug_Arah_Motor[1] == 0) {
        ledcWrite(motor2.pwm_ch1, 0);
        ledcWrite(motor2.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[1] == 1) {
        ledcWrite(motor2.pwm_ch1, abs(500));
        ledcWrite(motor2.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[1] == 2) {
        ledcWrite(motor2.pwm_ch1, 0);
        ledcWrite(motor2.pwm_ch2, abs(500));
      }

      debug_PWM_Motor[0] = abs(pwmOut1);
      debug_PWM_Motor[1] = abs(pwmOut2);
    }
  }


  //  if (MODE_ROBOT_NOW == ROBOT_RUN) {
  //
  //  }
  kirimFeedbackRPMMotor();
  Serial.println(MODE_ROBOT_NOW);
  //  kirimFeedbackPWMMotor();/

  //  kirimFeedbackDirectionMotor();/

  //  if (btn1 == 0) {
  //    while (btn1 == 0) btn1 = digitalRead(15);
  //  }
  //
  //  if (btn2 == 0) {
  //    while (btn2 == 0) btn2 = digitalRead(4);
  //  }
  //
  //  if (btn3 == 0) {
  //    while (btn3 == 0) btn3 = digitalRead(18);
  //  }
  //
  //  if (btn4 == 0) {
  //    while (btn4 == 0) btn4 = digitalRead(19);
  //
  //    delay(10);
}
