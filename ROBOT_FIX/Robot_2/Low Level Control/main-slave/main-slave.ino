#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#include "SerialTransfer.h"
#include <ArduinoJson.h>
#include <ESP32Encoder.h>
#include "ESP32TimerInterrupt.h"

#include "PID_Kontrol.h"
#include "Motor.h"
#include "FireTimer.h"

SerialTransfer keMaster;
FireTimer msTimer;
FireTimer timerNgeprint1, timerNgeprint2, timerNgeprint3, timerNgeprint4 ;

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

// Dari Master
volatile int rpm3, rpm4;

//Motor 1
volatile int lastCount1 = 0;
volatile int deltaCount1 = 0;
volatile int rpm1 = 0;
volatile int pwm1 = 0;

volatile int encCnt1;

//Motor 2
volatile int lastCount2 = 0;
volatile int deltaCount2 = 0;
volatile int rpm2 = 0;
volatile int pwm2 = 0;
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



// MODE ROBOT
#define DEBUG_ROS_PWM "debug_ros_pwm"
#define DEBUG_ROS_RPM "debug_ros_rpm"
#define DEBUG_ROBOT_PWM "debug_pwm"
#define DEBUG_ROBOT_RPM "debug_rpm"
#define ROBOT_RUN "robot_run"
#define ROBOT_STOP "robot_stop"

String MODE_ROBOT_NOW = ROBOT_STOP;

#define ROBOT_ID 1

/// ---- DEBUG MOTOR ROS ---- //
int debug_PWM_Motor[4] = {0, 0, 0, 0};
//int debug_PWM_Motor_L[4] = {0, 0, 0, 0};
int debug_Arah_Motor[4] = {0, 0, 0, 0};
float debug_RPM_Motor[4] = {0, 0, 0, 0};

// TIMER PID KALKULASI
void IRAM_ATTR TIMER_PID_M1(void) {
  pwmOut1 =  motor1_PID.kalkulasi(rpm1);
  //  if (pwmOut1 < 0) pwmOut1 = 0;
  //  if (pwmOut1 > 1023) pwmOut1 = 1023;
}
void IRAM_ATTR TIMER_PID_M2(void) {
  pwmOut2 =  motor2_PID.kalkulasi(rpm2);
  //  if (pwmOut2 < 0) pwmOut2 = 0;
  //  if (pwmOut2 > 1023) pwmOut2 = 1023;
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

int TombolMenu[4] = {15, 4, 18, 19};
int btn1, btn2, btn3, btn4;

const functionPtr callbackArr[] = { dari_master };

// INTERFACE
char TampilanAtas[][20] = {"Main Menu:", "Starting Robot"};
char TampilanBawahMain[][20] = {"> MODE ROBOT", "> INFO MOTOR"};
char TampilanBawahModeRobot[][20] = {"> ROBOT START",  "> ROBOT STOP", "> SET RPM", "> SET PID", "> SET PWM", "> EXIT" };
char TampilanBawahListMotor[][20] = {"> Motor 1",  "> Motor 2", "> Motor 3", "> Motor 4", "> EXIT"};
char TampilanAtasModeTuningPID[][20] = {"Tunning PID M1:",  "Tunning PID M2:", "Tunning PID M3:", "Tunning PID M4:", "  "};
char TampilanAtasModeTuningPWM[][20] = {"Tunning PWM M1:",  "Tunning PWM M2:", "Tunning PWM M3:", "Tunning PWM M4:", "  "};
char TampilanAtasModeTuningRPM[][20] = {"Tunning RPM M1:",  "Tunning RPM M2:", "Tunning RPM M3:", "Tunning RPM M4:", "  "};
char TampilanBawahModeTuningPID[][20] = {"> Set KP",  "> Set KI", "> Set KD", "> Set Ts", "> Set Ti", "> Set Td", "> EXIT" };
char TampilanBawahTuning[][20] = {":= ", "> OK", "> EXIT", " " };
char TampilanAtasTuningPID[][20] = {"Set Kp", "Set Ki", "Set Kd", "Set Ts", "Set Ti", "Set Td", "   "};



String flag_btn = "TBL_MAIN";
int counter_mainBtn = 0, counter_modeRobotBtn = 0, counter_listMotor = 0, counter_tuningPID = 0,
    counter_tuningPWM_INPUT = 0, counter_tuningPID_INPUT = 0, counter_tuningRPM_INPUT = 0;


float settingRPM_INPUT[5] = { 0.0, 0.0, 0.0, 0.0, 0.0}; // Setting Param INPUT
float settingRPM[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // RPM Variable

float settingPWM[5] = { 0.0, 0.0, 0.0, 0.0, 0.0};
float settingPWM_INPUT[5] = {0.0, 0.0, 0.0, 0.0, 0.0};


float settingPID[][10] = {
  {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // settingPID KP KI KD Ts Ti Td M1
  {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M2
  {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M3
  {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M4
};

float settingPID_INPUT[][10] = {
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M1
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M2
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M3
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // KP KI KD Ts Ti Td M4
};


void setup() {
  lcdInit();
  //  introDisplay();

  //// GPIO
  pinMode(TombolMenu[0], INPUT_PULLUP);
  pinMode(TombolMenu[1], INPUT_PULLUP);
  pinMode(TombolMenu[2], INPUT_PULLUP);
  pinMode(TombolMenu[3], INPUT_PULLUP);

  Serial.begin(9600);

  Serial2.begin(115200);

  keMasterConfig();
  configMotor();

  lcd.clear();
}

void loop() {
  keMaster.tick();


  //  kirimFeedbackPWMMotor();/
  //  kirimFeedbackDirectionMotor();/

  bacaTombol();
  tombolHandler();

  // INTERFACE DISPLAY / OUT
  interfaceMain();
  control_motor();

  //  ledcWrite(motor1.pwm_ch1, 1000);
  //  ledcWrite(motor1.pwm_ch2, 0);
  kirimInformasiKeMaster();
  //
  if (timerNgeprint1.fire() ) {
    Serial.println(MODE_ROBOT_NOW);

    //
    //    // PID
    //    for (int i = 0; i < 1; i++) {
    //      for (int j = 0; j < 6; j++) {
    //
    //        Serial.println("PID  " + String(i) + " Kp : " + String(settingPID[i][j]));
    //        Serial.println("PID  " + String(i) + " Ki : " + String(settingPID[i][j]));
    //        Serial.println("PID  " + String(i) + " Ki : " + String(settingPID[i][j]));
    //        Serial.println("PID  " + String(i) + " Ts : " + String(settingPID[i][j]));
    //        Serial.println("PID  " + String(i) + " Ti : " + String(settingPID[i][j]));
    //        Serial.println("PID  " + String(i) + " Td : " + String(settingPID[i][j]));
    //      }
    //    }
    //    // PWM
    //    for (int b = 0; b < 5; b++) {
    //      Serial.println("PWM " + String(b) + " : " + String(settingPWM[b]));
    //    }
    //
    //    // RPM
    //    for (int a = 0; a < 5; a++) {
    //      Serial.println("RPM " + String(a) + " : " + String(settingRPM[a]));
    //    }
    //
  }
}
