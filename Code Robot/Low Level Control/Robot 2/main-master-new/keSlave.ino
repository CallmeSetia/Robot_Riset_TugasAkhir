

void komunikasiSlave_Init() {
  
  ///////////////////////////////////////////////////////////////// Config Parameters
  configST myConfig;
  myConfig.debug        = false;
  myConfig.callbacks    = callbackArr;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////////////////////////////////////////////////////////////////
  timerNgeprint3.begin(5);
  timerNgeprint2.begin(5);

  keSlave.begin(Serial2, myConfig);
   //  Serial.begin(9600);  // Comment jika pakai ros
  Serial2.begin(115200); // KE Slave pakai baud 57600


}

void kirimInformasiKeSlave() {


  int mode_robot = getModeRobot();

  String paramPID;

  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 6; i++) {
      //     // Cek Param ESP Slave-Master dan Ros Sama Atau Tidak
      //     if (settingPID[j][i] != settingPID_ROS[j][i]) {
      //      settingPID[j][i] = settingPID_ROS[j][i];
      //     }

      paramPID += String(settingPID[j][i]);
      if (j < 3)  paramPID += String(",");
    }
    if (j < 3)  paramPID += String("|");
  }

  String paramPWM;
  for (int i = 0; i < 4; i++) {
    paramPWM += settingPWM[i];
    if (i < 3) paramPWM += String("|");
  }

  String paramRPM;
  for (int i = 0; i < 4; i++) {
    paramRPM += settingRPM[i];
    if (i < 3) paramRPM += String("|");
  }

  char data[500];
  uint16_t sendSize = 0;

  String printing = String(mode_robot) + "#" +
                    String(paramPID) + "!" + String(paramPWM) + "!" + String(paramRPM) + "#" +
                    String(rpm3) + "|" + String(pwm3) + "!" +  String(rpm4) + "|" + String(pwm4);


  printing.toCharArray(data, printing.length() + 1);
  sendSize = keSlave.txObj(data, sendSize);

   keSlave.sendData(sendSize);

}



void dariSlave()
{
  char arr[500];  char paramRobot[100];


  uint16_t recSize = 0;
  recSize = keSlave.rxObj(arr, recSize);
  String Read = String(arr);


  String ModeRobot = parseString(Read, "#", 0);
  String Param     = parseString(Read, "#", 1);


  String Feedback  = parseString(Read, "#", 2);

  String Feedback_M1 =  parseString(Feedback, "!", 0);
  String Feedback_M2 =  parseString(Feedback, "!", 1);

  String Feedback_M1_RPM =  parseString(Feedback_M1, "|", 0);
  String Feedback_M2_RPM =  parseString(Feedback_M2, "|", 0);



  // Param
  String PID_SetParam = parseString(Param, "!", 0 );
  for (int j = 2; j < 4; j++) {
    String PID_Motor = parseString(PID_SetParam, "|", j);
    for (int i = 0; i < 6; i++) {
      String PID_Motor_Param = parseString(PID_Motor, ",", i);
      settingPID_INPUT[j][i] = PID_Motor_Param.toFloat();
      settingPID[j][i] = PID_Motor_Param.toFloat();
    }
  }

  String PWM_SetParam = parseString(Param, "!", 1);
  for (int i = 0; i < 4; i++) {
    String PWM_Motor = parseString(PWM_SetParam, "|", i);
    settingPWM_INPUT[i] = PWM_Motor.toInt();


  }

  String RPM_SetParam = parseString(Param, "!", 2);
  for (int i = 0; i < 4; i++) {
    String RPM_Motor = parseString(RPM_SetParam, "|", i);
    settingRPM_INPUT[i] = RPM_Motor.toInt();
  }

  ParamPWM_Motor.x = settingPWM_INPUT[0];
  ParamPWM_Motor.y = settingPWM_INPUT[1];
  ParamPWM_Motor.z = settingPWM_INPUT[2];
  ParamPWM_Motor.w = settingPWM_INPUT[3];


  ParamRPM_Motor.x = settingRPM_INPUT[0];
  ParamRPM_Motor.y = settingRPM_INPUT[1];
  ParamRPM_Motor.z = settingRPM_INPUT[2];
  ParamRPM_Motor.w = settingRPM_INPUT[3];


//  ParamPID_Motor1.kp = settingPID_INPUT[0][0];
//  ParamPID_Motor1.ki = settingPID_INPUT[0][1];
//  ParamPID_Motor1.kd = settingPID_INPUT[0][2];
//  ParamPID_Motor1.ts = settingPID_INPUT[0][3];
//  ParamPID_Motor1.ti = settingPID_INPUT[0][4];
//  ParamPID_Motor1.td = settingPID_INPUT[0][5];
//
//  ParamPID_Motor1_Pub.publish(&ParamPID_Motor1);
//
//
//  ParamPID_Motor2.kp = settingPID_INPUT[1][0];
//  ParamPID_Motor2.ki = settingPID_INPUT[1][1];
//  ParamPID_Motor2.kd = settingPID_INPUT[1][2];
//  ParamPID_Motor2.ts = settingPID_INPUT[1][3];
//  ParamPID_Motor2.ti = settingPID_INPUT[1][4];
//  ParamPID_Motor2.td = settingPID_INPUT[1][5];
//
//  ParamPID_Motor2_Pub.publish(&ParamPID_Motor2);
//
//  ParamPID_Motor3.kp = settingPID_INPUT[2][0];
//  ParamPID_Motor3.ki = settingPID_INPUT[2][1];
//  ParamPID_Motor3.kd = settingPID_INPUT[2][2];
//  ParamPID_Motor3.ts = settingPID_INPUT[2][3];
//  ParamPID_Motor3.ti = settingPID_INPUT[2][4];
//  ParamPID_Motor3.td = settingPID_INPUT[2][5];
//
//  ParamPID_Motor3_Pub.publish(&ParamPID_Motor3);
//
//  ParamPID_Motor4.kp = settingPID_INPUT[3][0];
//  ParamPID_Motor4.ki = settingPID_INPUT[3][1];
//  ParamPID_Motor4.kd = settingPID_INPUT[3][2];
//  ParamPID_Motor4.ts = settingPID_INPUT[3][3];
//  ParamPID_Motor4.ti = settingPID_INPUT[3][4];
//  ParamPID_Motor4.td = settingPID_INPUT[3][5];
//
//  ParamPID_Motor4_Pub.publish(&ParamPID_Motor4);


  //    sprintf(msgbuf, "PARAM PWM %f %f %f %f", settingPWM[0], settingPWM[1], settingPWM[2], settingPWM[3]);
  //    Robot.loginfo(msgbuf);

  // MODE ROBOT
  MODE_ROBOT_NOW = ModeRobot.toInt();


  rpm1 = Feedback_M1_RPM.toInt();
  rpm2 = Feedback_M2_RPM.toInt();
}

//
//void kirimControlCommandKeSlave(){
//
//}
