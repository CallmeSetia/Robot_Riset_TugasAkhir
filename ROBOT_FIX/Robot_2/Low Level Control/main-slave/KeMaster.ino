
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


  String ModeRobot = parseString(Read, "#", 0);
  String Param     = parseString(Read, "#", 1);
  String Feedback  = parseString(Read, "#", 2);

  String Feedback_M3 =  parseString(Feedback, "!", 0);
  String Feedback_M4 =  parseString(Feedback, "!", 1);

  String Feedback_M3_RPM =  parseString(Feedback_M3, "|", 0);
  String Feedback_M4_RPM =  parseString(Feedback_M4, "|", 0);

  rpm3 = Feedback_M3_RPM.toInt();
  rpm4 = Feedback_M4_RPM.toInt();

  //  Serial.println(Feedback_M3_RPM);
  //  Serial.println(Feedback_M4_RPM);
  //  0|0!0|0

}

void kirimInformasiKeMaster() {
  int mode_robot = getModeRobot();
  String paramPID;
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 6; i++) {
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
                    String(rpm1) + "|" + String(pwm1) + "!" +  String(rpm2) + "|" + String(pwm2);


  printing.toCharArray(data, printing.length() + 1);
  sendSize = keMaster.txObj(data, sendSize);

  if (timerNgeprint3.fire()) {
    keMaster.sendData(sendSize);
  }
  //  delay()
}
