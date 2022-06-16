
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
