/// ---- DEBUG MOTOR ROS ---- //
int debug_PWM_Motor[4] = {0, 0, 0, 0};
//int debug_PWM_Motor_L[4] = {0, 0, 0, 0};
int debug_Arah_Motor[4] = {0, 0, 0, 0};
float debug_RPM_Motor[4] = {0, 0, 0, 0};
float settingRPM[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // RPM Variable
float settingRPM_INPUT[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // RPM Variable

float settingPWM[5] = { 0.0, 0.0, 0.0, 0.0, 0.0};
float settingPWM_INPUT[5] = { 0.0, 0.0, 0.0, 0.0, 0.0};

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



void Motor_Init() {
   // TIMER BACA ENCODER
  TimerPID3.attachInterruptInterval(50 * 1000, TIMER_PID_M3);
  TimerPID4.attachInterruptInterval(50 * 1000, TIMER_PID_M4);

  ITimer3.attachInterruptInterval(100 * 1000, TimerHandler3);
  ITimer4.attachInterruptInterval(100 * 1000, TimerHandler4);

  ESP32Encoder::useInternalWeakPullResistors = UP;   // Attache pins for use as encoder pins

  // ---- MOTOR 3
  motor3.pin(32, 33);
  motor3.enc(36, 39);
  motor3.pwm_ch(0, 1);

  // --- MOTOR PID
  motor3_PID.mulai();
  //  motor3_PID.setSetPoints(80);
  motor3_PID.setSampling(50, 50);
  motor3_PID.setKonstanta(1.2, 0, 0);

  // --- MOTOR 3 PWM
  ledcSetup(motor3.pwm_ch1, freq, resolution);
  ledcSetup(motor3.pwm_ch2, freq, resolution);

  ledcAttachPin(motor3.pin1, motor3.pwm_ch1);
  ledcAttachPin(motor3.pin2, motor3.pwm_ch2);
  // -------------------------------------

  // ---- MOTOR 4
  motor4.pin(25, 26);
  motor4.enc(35, 34);
  motor4.pwm_ch(2, 3);

  // --- MOTOR 4 PID
  motor4_PID.mulai();
  motor4_PID.setSetPoints(80);
  motor4_PID.setSampling(0.1, 0.1);
  motor4_PID.setKonstanta(1, 0, 0);

  // --- MOTOR 4 PWM
  ledcSetup(motor4.pwm_ch1, freq, resolution);
  ledcSetup(motor4.pwm_ch2, freq, resolution);

  ledcAttachPin(motor4.pin1, motor4.pwm_ch1);
  ledcAttachPin(motor4.pin2, motor4.pwm_ch2);
  // -------------------------------------

  encoder3.attachHalfQuad(motor3.enc1, motor3.enc2); //encoderA, encoderB
  encoder4.attachHalfQuad(motor4.enc1, motor4.enc2); //encoderA, encoderB

  // count awal - start
  encoder3.setCount(0);
  encoder4.setCount(0);

}

void controlMotor() {
  settingDirection();
  String ModeRobot = parseModeRobot(MODE_ROBOT_NOW.toInt());

  if (ModeRobot == DEBUG_ROBOT_PWM) {

    if (enablePID == 1) {
      enablePID = 0; // Disable PID Flag
      motor3_PID.reset();
      motor4_PID.reset();
    }
    Serial.println("Arah Control");
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

  if (ModeRobot == DEBUG_ROBOT_RPM) {
    if (enablePID == 0) {
      enablePID = 1;
      motor3_PID.mulai();
      motor4_PID.mulai(); //

    }

    motor3_PID.setSetPoints(settingRPM[2]);
    motor3_PID.setSampling(settingPID[2][4], settingPID[2][5]); // settingPID KP KI KD Ts Ti Td M3
    motor3_PID.setKonstanta(settingPID[2][0], settingPID[2][1], settingPID[2][2]); // // settingPID KP KI KD Ts Ti Td M3

    //    motor3_PID.setSampling(10, 10); // settingPID KP KI KD Ts Ti Td M3
    //    motor3_PID.setKonstanta(1, 0.15, 0); // // settingPID KP KI KD Ts Ti Td M3

    motor4_PID.setSetPoints(settingRPM[3]);
    motor4_PID.setSampling(settingPID[3][4], settingPID[3][5]); // settingPID KP KI KD Ts Ti Td M4
    motor4_PID.setKonstanta(settingPID[3][0], settingPID[3][1], settingPID[3][2]);
    //
    //    motor4_PID.setSampling(10, 10); // settingPID KP KI KD Ts Ti Td M3
    //    motor4_PID.setKonstanta(0.3, 0.15, 0); // // settingPID KP KI KD Ts Ti Td M3


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


        //        Robot.loginfo("PID ENABLE 0");
      }
      if (debug_Arah_Motor[2] == 1) {
        ledcWrite(motor3.pwm_ch1, abs(pwmOut3));
        ledcWrite(motor3.pwm_ch2, 0);


        //        Robot.loginfo("PID ENABLE 1");
      }
      if (debug_Arah_Motor[2] == 2) {
        ledcWrite(motor3.pwm_ch1, 0);
        ledcWrite(motor3.pwm_ch2, abs(pwmOut3));


        //        Robot.loginfo("PID ENABLE 2");
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

    //    sprintf(msgbuf, "PARAM PWM %f %f %f %f", settingRPM[2], settingRPM[3], pwmOut3, pwmOut4);
    //    Robot.loginfo(msgbuf);
    pwm3 = abs(pwmOut3);
    pwm4 = abs(pwmOut4);

  }

  if (ModeRobot == ROBOT_STOP) {
    enablePID = 0; // Disable PID Flag
    motor3_PID.reset();
    motor4_PID.reset();

    ledcWrite(motor3.pwm_ch1, 0);
    ledcWrite(motor3.pwm_ch2, 0);

    ledcWrite(motor4.pwm_ch1, 0);
    ledcWrite(motor4.pwm_ch2, 0);
  }

}

void settingDirection() {
  String ModeRobot = parseModeRobot(MODE_ROBOT_NOW.toInt());
  //  Serial.println(ModeRobot);/

  if (ModeRobot == DEBUG_ROBOT_PWM) {

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

  if (ModeRobot == DEBUG_ROBOT_RPM) {
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
