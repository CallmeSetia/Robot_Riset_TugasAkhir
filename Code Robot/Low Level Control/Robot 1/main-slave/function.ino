
//Serial Komunikasi

// -- FUNCTION BIASA

void kirimFeedbackRPMMotor() {

  delay(10);
  uint16_t sendSize = 0;
  char buf[200];

  String printing = "ros_rpm#" + String(rpm1) + "#" + String(rpm2);
  printing.toCharArray(buf, printing.length() + 1);
  //  Serial.println(buf);/
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

int getModeRobot() {

  if ( MODE_ROBOT_NOW == ROBOT_RUN) return 1;
  if ( MODE_ROBOT_NOW == ROBOT_STOP) return 2;
  if ( MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) return 3;
  if ( MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) return 4;
}

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
