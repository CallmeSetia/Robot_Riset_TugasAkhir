void controlMotor() {
  settingDirection();

  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {  // Mode Setting/Debug Robot Melalui ROS MASTER

    if (enablePID == 1) {
      enablePID = 0; // Disable PID Flag
      motor3_PID.reset();
      motor4_PID.reset();
    }

    String printing = "ros_pwm#1#" + String(debug_Arah_Motor[0]) + "#" + String(debug_PWM_Motor[0])
                      + "#2#" + String(debug_Arah_Motor[1]) + "#" + String(debug_PWM_Motor[1]);

    uint16_t sendSize = 0;
    char buf[200];

    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);
    delay(30);

    sendSize = 0;
    buf[200];

    printing = "mode#" + String(DEBUG_ROS_PWM);
    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);

    delay(30);

    // Motor 3
    if (debug_Arah_Motor[2] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor3.pwm_ch1, 0);
      ledcWrite(motor3.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[2] == 1) {
      ledcWrite(motor3.pwm_ch1, debug_PWM_Motor[2]);
      ledcWrite(motor3.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[2] == 2) {
      ledcWrite(motor3.pwm_ch1, 0);
      ledcWrite(motor3.pwm_ch2, debug_PWM_Motor[2]);
    }

    // Motor 4
    if (debug_Arah_Motor[3] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor4.pwm_ch1, 0);
      ledcWrite(motor4.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[3] == 1) {
      ledcWrite(motor4.pwm_ch1, debug_PWM_Motor[3]);
      ledcWrite(motor4.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[3] == 2) {
      ledcWrite(motor4.pwm_ch1, 0);
      ledcWrite(motor4.pwm_ch2, debug_PWM_Motor[3]);
    }

  }
  // RPM - PID
  if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) { //
    if (enablePID == 0) {
      enablePID = 1;
      motor3_PID.mulai();
      motor4_PID.mulai(); ///
    }
    String printing = "ros_rpm#1#" + String(debug_Arah_Motor[0]) + "#" + String(debug_RPM_Motor[0])
                      + "#2#" + String(debug_Arah_Motor[1]) + "#" + String(debug_RPM_Motor[1]);

    uint16_t sendSize = 0;
    char buf[200];

    printing.toCharArray(buf, printing.length() + 1);

    sendSize = keSlave.txObj(buf, sendSize);
    keSlave.sendData(sendSize);
    delay(30);

    motor3_PID.setSetPoints(debug_RPM_Motor[2]);
    motor4_PID.setSetPoints(0);


    if (enablePID == 1) {

      //      pwmOut4 = motor4_PID.kalkulasi(abs(rpm4));

      if (debug_Arah_Motor[2] == 0) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[2] == 1) {
        ledcWrite(motor3.pwm_ch1, abs(pwmOut3));
        ledcWrite(motor3.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[2] == 2) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, abs(pwmOut3));
      }

      if (debug_Arah_Motor[3] == 0) {
        ledcWrite(motor4.pwm_ch1, 0);
        ledcWrite(motor4.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[3] == 1) {
        ledcWrite(motor4.pwm_ch1, abs(pwmOut4));
        ledcWrite(motor4.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[3] == 2) {
        ledcWrite(motor4.pwm_ch1, 0);
        ledcWrite(motor4.pwm_ch2, abs(pwmOut4));
      }

      debug_PWM_Motor[2] = abs(pwmOut3);
      debug_PWM_Motor[3] = abs(pwmOut4);
    }

  }

  if (MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {

    if (enablePID == 1) {
      enablePID = 0; // Disable PID Flag
      motor3_PID.reset();
      motor4_PID.reset();
      TimerPID3.stopTimer();
      TimerPID4.stopTimer();
    }
    // Motor 3
    if (debug_Arah_Motor[2] == 0) {
      ledcWrite(motor3.pwm_ch1, 0);
      ledcWrite(motor3.pwm_ch2, 0);
    }
    if (debug_Arah_Motor[2] == 1) {
      ledcWrite(motor3.pwm_ch1, abs(settingPWM[2]));
      ledcWrite(motor3.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[2] == 2) {
      ledcWrite(motor3.pwm_ch1, 0);
      ledcWrite(motor3.pwm_ch2, abs(settingPWM[2]));
    }

    // Motor 4
    if (debug_Arah_Motor[3] == 0) { // Direction = 0 --> Mati
      ledcWrite(motor4.pwm_ch1, 0);
      ledcWrite(motor4.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[3] == 1) {

      ledcWrite(motor4.pwm_ch1, abs(settingPWM[3]));
      ledcWrite(motor4.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[3] == 2) {
      ledcWrite(motor4.pwm_ch1, 0);
      ledcWrite(motor4.pwm_ch2, abs(settingPWM[3]));
    }

    pwm3 = abs(settingPWM[2]);
    pwm4 = abs(settingPWM[3]);

  }

  if (MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) {
    if (enablePID == 0) {
      enablePID = 1;
      motor3_PID.mulai();
      motor4_PID.mulai(); //

      TimerPID3.attachInterruptInterval(settingPID[2][3] * 1000, TIMER_PID_M3);
      TimerPID4.attachInterruptInterval(settingPID[3][3] * 1000, TIMER_PID_M4);

    }

    motor3_PID.setSetPoints(settingRPM[2]);
    motor3_PID.setSampling(settingPID[2][4], settingPID[2][5]); // settingPID KP KI KD Ts Ti Td M3
    motor3_PID.setKonstanta(settingPID[2][0], settingPID[2][1], settingPID[2][2]); // // settingPID KP KI KD Ts Ti Td M3

    motor4_PID.setSetPoints(settingRPM[3]);
    motor4_PID.setSampling(settingPID[3][4], settingPID[3][5]); // settingPID KP KI KD Ts Ti Td M4
    motor4_PID.setKonstanta(settingPID[3][0], settingPID[3][1], settingPID[3][2]);

    if (enablePID == 1) {


      pwmOut3 =  abs(pwmOut3);
      pwmOut4 =  abs(pwmOut4);

      // PWM OUT 3 FILTERING
      if (pwmOut3 <= 0) {
        pwmOut3 = 0;
      }
      else if (pwmOut3 >= 1023) {
        pwmOut3 = 1023;
      }

      // PWM OUT 4 FILTERING
      if (pwmOut4 <= 0) {
        pwmOut4 = 0;
      }
      else if (pwmOut4 >= 1023) {
        pwmOut4 = 1023;
      }

      // Motor 3
      if (debug_Arah_Motor[2] == 0) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[2] == 1) {
        ledcWrite(motor3.pwm_ch1, abs(pwmOut3));
        ledcWrite(motor3.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[2] == 2) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, abs(pwmOut3));
      }

      // Motor 4

      if (debug_Arah_Motor[3] == 0) {
        ledcWrite(motor4.pwm_ch1, 0);
        ledcWrite(motor4.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[3] == 1) {
        ledcWrite(motor4.pwm_ch1, abs(pwmOut4));
        ledcWrite(motor4.pwm_ch2, 0);
      }
      if (debug_Arah_Motor[3] == 2) {
        ledcWrite(motor4.pwm_ch1, 0);
        ledcWrite(motor4.pwm_ch2, abs(pwmOut4));
      }
    }
    pwm3 = abs(pwmOut3);
    pwm4 = abs(pwmOut4);

  }

  if (MODE_ROBOT_NOW == ROBOT_STOP) {
    enablePID = 0; // Disable PID Flag
    motor3_PID.reset();
    motor4_PID.reset();
    TimerPID3.stopTimer();
    TimerPID4.stopTimer();

    ledcWrite(motor3.pwm_ch1, 0);
    ledcWrite(motor3.pwm_ch2, 0);

    ledcWrite(motor4.pwm_ch1, 0);
    ledcWrite(motor4.pwm_ch2, 0);
  }

}

void settingDirection() {
  if (MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {
    // Seting Direction PWM
    if (settingPWM[2] == 0) {
      debug_Arah_Motor[2] = 0;
    }
    else if (settingPWM[2] < 0) {
      debug_Arah_Motor[2] = 1;
    }
    else if (settingPWM[2] > 0) {
      debug_Arah_Motor[2] = 2;
    }

    if (settingPWM[3] == 0) {
      debug_Arah_Motor[3] = 0;
    }
    else if (settingPWM[3] < 0) {
      debug_Arah_Motor[3] = 1;
    }
    else if (settingPWM[3] > 0) {
      debug_Arah_Motor[3] = 2;
    }
  }

  if (MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) {
    // Seting Direction RPM
    if (settingRPM[2] == 0) {
      debug_Arah_Motor[2] = 0;
    }
    else if (settingRPM[2] < 0) {
      debug_Arah_Motor[2] = 2;
    }
    else if (settingRPM[2] > 0) {
      debug_Arah_Motor[2] = 1;
    }

    if (settingRPM[3] == 0) {
      debug_Arah_Motor[3] = 0;
    }
    else if (settingRPM[3] < 0) {
      debug_Arah_Motor[3] = 2;
    }
    else if (settingRPM[3] > 0) {
      debug_Arah_Motor[3] = 1;
    }
  }

}
