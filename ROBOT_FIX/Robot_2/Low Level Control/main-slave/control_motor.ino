void control_motor() {
  settingDirection();
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
        ledcWrite(motor1.pwm_ch1, abs(pwmOut1));
        ledcWrite(motor1.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[0] == 2) {
        ledcWrite(motor1.pwm_ch1, 0);
        ledcWrite(motor1.pwm_ch2, abs(pwmOut1));
      }

      // Motor 2
      if (debug_Arah_Motor[1] == 0) {
        ledcWrite(motor2.pwm_ch1, 0);
        ledcWrite(motor2.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[1] == 1) {
        ledcWrite(motor2.pwm_ch1, abs(pwmOut2));
        ledcWrite(motor2.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[1] == 2) {
        ledcWrite(motor2.pwm_ch1, 0);
        ledcWrite(motor2.pwm_ch2, abs(pwmOut2));
      }

      debug_PWM_Motor[0] = abs(pwmOut1);
      debug_PWM_Motor[1] = abs(pwmOut2);
    }
  }
  if (MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {

    if (enablePID == 1) {
      enablePID = 0; // Disable PID Flag
      motor1_PID.reset();
      motor2_PID.reset();
    }
    // Motor 1
    if (debug_Arah_Motor[0] == 0) {
      ledcWrite(motor1.pwm_ch1, 0);
      ledcWrite(motor1.pwm_ch2, 0);
    }
    if (debug_Arah_Motor[0] == 1) {
      ledcWrite(motor1.pwm_ch1, abs(settingPWM[0]));
      ledcWrite(motor1.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[0] == 2) {
      ledcWrite(motor1.pwm_ch1, 0);
      ledcWrite(motor1.pwm_ch2, abs(settingPWM[0]));
    }

    // Motor 2
    if (debug_Arah_Motor[1] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor2.pwm_ch1, 0);
      ledcWrite(motor2.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[1] == 1) {

      ledcWrite(motor2.pwm_ch1, abs(settingPWM[1]));
      ledcWrite(motor2.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[1] == 2) {
      ledcWrite(motor2.pwm_ch1, 0);
      ledcWrite(motor2.pwm_ch2, abs(settingPWM[1]));
    }

    //    delay(10);
    //    uint16_t sendSize = 0;
    //    char buf[200];
    //
    //    String printing = "robot_pwm#" + String(settingPWM[2]) + "#" + String(settingPWM[3]);
    //    printing.toCharArray(buf, printing.length() + 1);
    //    sendSize = keMaster.txObj(buf, sendSize);
    //    keMaster.sendData(sendSize);
  }
  if (MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) {
    if (enablePID == 0) {
      enablePID = 1;
      motor1_PID.mulai();
      motor2_PID.mulai(); //
    }
    Serial.println("oe");
    motor1_PID.setSetPoints(500);
    motor2_PID.setSetPoints(500);

    if (enablePID == 1) {

      // Motor 1
      if (debug_Arah_Motor[0] == 0) {
        ledcWrite(motor1.pwm_ch1, 0);
        ledcWrite(motor1.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[0] == 1) {
        ledcWrite(motor1.pwm_ch1, abs(pwmOut1));
        ledcWrite(motor1.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[0] == 2) {
        ledcWrite(motor1.pwm_ch1, 0);
        ledcWrite(motor1.pwm_ch2, abs(pwmOut1));
      }

      // Motor 2

      if (debug_Arah_Motor[1] == 0) {
        ledcWrite(motor2.pwm_ch1, 0);
        ledcWrite(motor2.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[1] == 1) {
        ledcWrite(motor2.pwm_ch1, abs(pwmOut2));
        ledcWrite(motor2.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[1] == 2) {
        ledcWrite(motor2.pwm_ch1, 0);
        ledcWrite(motor2.pwm_ch2, abs(pwmOut2));
      }

      //      debug_PWM_Motor[0] = abs(pwmOut1);
      //      debug_PWM_Motor[1] = abs(pwmOut2);
    }


  }
  if (MODE_ROBOT_NOW == ROBOT_STOP) {
    ledcWrite(motor1.pwm_ch1, 0);
    ledcWrite(motor1.pwm_ch2, 0);
  }

}

void settingDirection() {
  if (MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {
    // Seting Direction PWM
    if (settingPWM[0] == 0) {
      debug_Arah_Motor[0] = 0;
    }
    else if (settingPWM[0] < 0) {
      debug_Arah_Motor[0] = 1;
    }
    else if (settingPWM[0] > 0) {
      debug_Arah_Motor[0] = 2;
    }

    if (settingPWM[1] == 0) {
      debug_Arah_Motor[1] = 0;
    }
    else if (settingPWM[1] < 0) {
      debug_Arah_Motor[1] = 1;
    }
    else if (settingPWM[1] > 0) {
      debug_Arah_Motor[1] = 2;
    }
  }

  if (MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) {
    // Seting Direction RPM
    if (settingRPM[0] == 0) {
      debug_Arah_Motor[0] = 0;
    }
    else if (settingRPM[0] < 0) {
      debug_Arah_Motor[0] = 1;
    }
    else if (settingRPM[0] > 0) {
      debug_Arah_Motor[0] = 2;
    }

    if (settingRPM[1] == 0) {
      debug_Arah_Motor[1] = 0;
    }
    else if (settingRPM[1] < 0) {
      debug_Arah_Motor[1] = 1;
    }
    else if (settingRPM[1] > 0) {
      debug_Arah_Motor[1] = 2;
    }
  }
}
