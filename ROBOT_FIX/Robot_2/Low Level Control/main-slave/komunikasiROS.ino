


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
      MODE_ROBOT_NOW = DEBUG_ROS_PWM;
    }
    else if (read2 == DEBUG_ROS_RPM) {
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

  if (read1 == "feedback_master") { //  "feedback_master#" + String(rpm3) + "#" + String(rpm4);
    if ( MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {
      //      Serial.println("Dari Master" + String(Read));
      read2 = parseString(Read, "#", 1); // RPM 3
      read3 = parseString(Read, "#", 2); // RPM 4
     
      int rpmMotor3 = read2.toInt(),
          rpmMotor4 = read3.toInt();

      debug_RPM_Motor[2] = rpmMotor3;
      debug_RPM_Motor[3] = rpmMotor4;

    }
  }
  delay(5);
}


void keMasterConfig() {
    ///////////////////////////////////////////////////////////////// Config Parameters
  configST myConfig;
  myConfig.debug        = true;
  myConfig.callbacks    = callbackArr;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);
  /////////////////////////////////////////////////////////////////
  keMaster.begin(Serial2, myConfig);

}
