

void kirimInformasiKeSlave() {


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
                    String(rpm3) + "|" + String(pwm3) + "!" +  String(rpm4) + "|" + String(pwm4);


  printing.toCharArray(data, printing.length() + 1);
  sendSize = keSlave.txObj(data, sendSize);

  if (timerNgeprint3.fire()) {
    keSlave.sendData(sendSize);
  }

}



void dariSlave()
{
  char arr[500];
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

  rpm1 = Feedback_M1_RPM.toInt();
  rpm2 = Feedback_M2_RPM.toInt();
}
