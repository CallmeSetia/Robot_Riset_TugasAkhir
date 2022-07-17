

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
      Serial.println(1);
      ledcWrite(motor1.pwm_ch1, 0);
      ledcWrite(motor1.pwm_ch2, 0);
    }
    if (debug_Arah_Motor[0] == 1) {
      Serial.println(2);
      ledcWrite(motor1.pwm_ch1, abs(settingPWM[0]));
      ledcWrite(motor1.pwm_ch2, 0);
    }

    else if (debug_Arah_Motor[0] == 2) {
      Serial.println(3);
      ledcWrite(motor1.pwm_ch1, 0);
      ledcWrite(motor1.pwm_ch2, abs(settingPWM[0]));
    }
    Serial.println(abs(settingPWM[0]));

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
    pwm1 = abs(settingPWM[0]);
    pwm2 = abs(settingPWM[1]);

  }

  if (MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) {
    if (enablePID == 0) {
      enablePID = 1;
      motor1_PID.mulai();
      motor2_PID.mulai(); //

    }

    
    motor1_PID.setSetPoints(settingRPM[0]);
    motor1_PID.setSampling(settingPID[0][4], settingPID[0][5]); // settingPID KP KI KD Ts Ti Td M1
    motor1_PID.setKonstanta(settingPID[0][0], settingPID[0][1], settingPID[0][2]); // // settingPID KP KI KD Ts Ti Td M1

    motor2_PID.setSetPoints(settingRPM[1]);
    motor2_PID.setSampling(settingPID[1][4], settingPID[1][5]); // settingPID KP KI KD Ts Ti Td M1
    motor2_PID.setKonstanta(settingPID[1][0], settingPID[1][1], settingPID[1][2]);


    if (enablePID == 1) {


      pwmOut1 =  abs(pwmOut1);
      pwmOut2 =  abs(pwmOut2);

      // PWM OUT 1 FILTERING
      if (pwmOut1 <= 0) {
        pwmOut1 = 0;
      }
      else if (pwmOut1 >= 1023) {
        pwmOut1 = 1023;
      }

      // PWM OUT 2 FILTERING
      if (pwmOut2 <= 0) {
        pwmOut2 = 0;
      }
      else if (pwmOut2 >= 1023) {
        pwmOut2 = 1023;
      }



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
    }
    pwm1 = abs(pwmOut1);
    pwm2 = abs(pwmOut2);

  }

  if (MODE_ROBOT_NOW == ROBOT_STOP) {
    enablePID = 0; // Disable PID Flag
    motor1_PID.reset();
    motor2_PID.reset();

    ledcWrite(motor1.pwm_ch1, 0);
    ledcWrite(motor1.pwm_ch2, 0);

    ledcWrite(motor2.pwm_ch1, 0);
    ledcWrite(motor2.pwm_ch2, 0);
  }

}

void settingDirection() {
  if (MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) {
    Serial.println("Arah Motor");
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

    Serial.println(String(debug_Arah_Motor[0]) +  String(debug_Arah_Motor[1]));
  }

  if (MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) {
    // Seting Direction RPM
    if (settingRPM[0] == 0) {
      debug_Arah_Motor[0] = 0;
    }
    else if (settingRPM[0] < 0) {
      debug_Arah_Motor[0] = 2;
    }
    else if (settingRPM[0] > 0) {
      debug_Arah_Motor[0] = 1;
    }

    if (settingRPM[1] == 0) {
      debug_Arah_Motor[1] = 0;
    }
    else if (settingRPM[1] < 0) {
      debug_Arah_Motor[1] = 2;
    }
    else if (settingRPM[1] > 0) {
      debug_Arah_Motor[1] = 1;
    }
  }
}
