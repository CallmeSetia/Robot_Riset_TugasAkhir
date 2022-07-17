// Dari Master
String read1, read2, read3, read4;
String bacaData2;
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
int debug_Arah_Motor[4] = {0, 0, 0, 0};
float debug_RPM_Motor[4] = {0, 0, 0, 0};

float settingRPM_INPUT[5] = { 0.0, 0.0, 0.0, 0.0, 0.0}; // Setting Param INPUT
float settingRPM[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // RPM Variable

float settingPWM[5] = { 0.0, 0.0, 0.0, 0.0, 0.0};
float settingPWM_INPUT[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

float settingStart[5] = { 0.0, 0.0, 0.0};
float settingStart_INPUT[5] = {0.0, 0.0, 0.0};

float settingPID[][10] = {
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // settingPID KP KI KD Ts Ti Td M1
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // KP KI KD Ts Ti Td M2
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // settingPID KP KI KD Ts Ti Td M1
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // KP KI KD Ts Ti Td M2
};

float settingPID_INPUT[][10] = {
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // settingPID KP KI KD Ts Ti Td M1
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // KP KI KD Ts Ti Td M2
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // settingPID KP KI KD Ts Ti Td M1
  {1.5, 2.3, 0.0, 0.0, 100, 100, 0.0, 0.0}, // KP KI KD Ts Ti Td M2
};

const functionPtr callbackArr[] = { dari_master };

void keMasterConfig() {
  ///////////////////////////////////////////////////////////////// Config Parameters
  configST myConfig;
  myConfig.debug        = false;
  myConfig.callbacks    = callbackArr;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////////////////////////////////////////////////////////////////
  keMaster.begin(Serial2, myConfig);

}
void dari_master()
{
  char arr[500];
  uint16_t recSize = 0;
  recSize = keMaster.rxObj(arr, recSize);
  String Read = String(arr);

  Serial.println(Read);
  String ModeRobot = parseString(Read, "#", 0);
  String Param     = parseString(Read, "#", 1);

  String Command_PWM = parseString(Param, "!", 1);
  String Command_RPM = parseString(Param, "!", 2);

  String Command_PWM_M1 = parseString(Command_PWM, "|", 0);
  String Command_PWM_M2 = parseString(Command_PWM, "|", 1);
  String Command_PWM_M3 = parseString(Command_PWM, "|", 2);
  String Command_PWM_M4 = parseString(Command_PWM, "|", 3);

  String Command_RPM_M1 = parseString(Command_RPM, "|", 0);
  String Command_RPM_M2 = parseString(Command_RPM, "|", 1);
  String Command_RPM_M3 = parseString(Command_RPM, "|", 2);
  String Command_RPM_M4 = parseString(Command_RPM, "|", 3);

  //  Serial.println("CMD" + Command_PWM_M3);
  // Ambil Parameter

  if (Command_PWM_M1.toInt() != settingPWM[0] ||  Command_PWM_M1.toInt() != settingPWM_INPUT[0]) {
    settingPWM[0] = Command_PWM_M1.toInt();

  }
  if (Command_PWM_M2.toInt() != settingPWM[1] ||  Command_PWM_M2.toInt() != settingPWM_INPUT[1]) {
    settingPWM[1] = Command_PWM_M2.toInt();

  }
  if (Command_PWM_M3.toInt() != settingPWM[2] ||  Command_PWM_M3.toInt() != settingPWM_INPUT[2]) {
    settingPWM[2] = Command_PWM_M3.toInt();

  }
  if (Command_PWM_M4.toInt() != settingPWM[3] ||  Command_PWM_M4.toInt() != settingPWM_INPUT[3]) {
    settingPWM[3] = Command_PWM_M4.toInt();

  }


  ////
  if (Command_RPM_M1.toInt() != settingRPM[0] ||  Command_RPM_M1.toInt() != settingRPM_INPUT[0]) {
    settingRPM[0] = Command_RPM_M1.toInt();
  }
  if (Command_RPM_M2.toInt() != settingRPM[1] ||  Command_RPM_M2.toInt() != settingRPM_INPUT[1]) {
    settingRPM[1] = Command_RPM_M2.toInt();
  }
  if (Command_RPM_M3.toInt() != settingRPM[2] ||  Command_RPM_M3.toInt() != settingRPM_INPUT[2]) {
    settingRPM[2] = Command_RPM_M3.toInt();
  }
  if (Command_RPM_M4.toInt() != settingRPM[3] ||  Command_RPM_M4.toInt() != settingRPM_INPUT[3]) {
    settingRPM[3] = Command_RPM_M4.toInt();
  }

  // if rpmMaster[0] != rpmSlave[0]
  // rpmSlave[0] = rpmMaster; rpmSlaveInput[0]=rpmMaster[0]


  String Feedback  = parseString(Read, "#", 2);

  String Feedback_M3 =  parseString(Feedback, "!", 0);
  String Feedback_M4 =  parseString(Feedback, "!", 1);

  String Feedback_M3_RPM =  parseString(Feedback_M3, "|", 0);
  String Feedback_M4_RPM =  parseString(Feedback_M4, "|", 0);

  rpm3 = Feedback_M3_RPM.toInt();
  rpm4 = Feedback_M4_RPM.toInt();



  // Cek Parameter dari ESP Slave Input dan Var Sama Atau Tidak
  //

  //  Serial.println(Feedback_M3_RPM);
  //  Serial.println(Feedback_M4_RPM);
  //  0|0!0|0

}

void kirimInformasiKeMaster() {
  int mode_robot = getModeRobot();
  String paramPID;
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 6; i++) {
      paramPID += String(settingPID_INPUT[j][i]);
      if (j < 4 && i < 5)  paramPID += String(",");
    }
    if (j < 3)  paramPID += String("|");
  }

  String paramPWM;
  for (int i = 0; i < 4; i++) {
    paramPWM += settingPWM_INPUT[i];
    if (i < 3) paramPWM += String("|");
  }

  String paramRPM;
  for (int i = 0; i < 4; i++) {
    paramRPM += settingRPM_INPUT[i];
    if (i < 3) paramRPM += String("|");
  }

  char data[500];
  uint16_t sendSize = 0;

  String printing = String(mode_robot) + "#" +
                    String(paramPID) + "!" + String(paramPWM) + "!" + String(paramRPM) + "#" +
                    String(rpm1) + "|" + String(pwm1) + "!" +  String(rpm2) + "|" + String(pwm2);


  printing.toCharArray(data, printing.length() + 1);

  sendSize = keMaster.txObj(data, sendSize);

  if (timerNgeprint3.fire()) {
    keMaster.sendData(sendSize);
  }
  //  delay()
}
