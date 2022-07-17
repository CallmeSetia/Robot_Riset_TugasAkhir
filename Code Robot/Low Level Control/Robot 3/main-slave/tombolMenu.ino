
int TombolMenu[4] = {15, 4, 18, 19};
int btn1, btn2, btn3, btn4;



void tombol_init() {
  //// GPIO
  pinMode(TombolMenu[0], INPUT_PULLUP);
  pinMode(TombolMenu[1], INPUT_PULLUP);
  pinMode(TombolMenu[2], INPUT_PULLUP);
  pinMode(TombolMenu[3], INPUT_PULLUP);

}
int bacaTombol() {
  btn1 = digitalRead(TombolMenu[0]);
  btn2 = digitalRead(TombolMenu[1]);
  btn3 = digitalRead(TombolMenu[2]);
  btn4 = digitalRead(TombolMenu[3]);
}

void tombolHandler() {
  // Tombol 1 Ditekan
  if (btn1 == 0) {
    while (btn1 == 0) btn1 = digitalRead(TombolMenu[0]);
    lcd.clear();

    if (flag_btn == "TBL_MAIN") {
      if (counter_mainBtn < 1) counter_mainBtn++;
      else counter_mainBtn = 0;
    }

    if (flag_btn == "TBL_MOTOR_START") {
      if (counter_StartRobot < 3) counter_StartRobot++;
      else counter_StartRobot = 0;
    }
    if (flag_btn == "TBL_MOTOR_START_INPUT") {
      if (counter_StartRobot_INPUT < 2) counter_StartRobot_INPUT++;
      else counter_StartRobot_INPUT = 0;
    }

    if (flag_btn == "TBL_MODE_ROBOT") {
      if (counter_modeRobotBtn < 5) counter_modeRobotBtn++;
      else counter_modeRobotBtn = 0;
    }

    if (flag_btn == "TBL_LIST_MOTOR") {
      if (counter_listMotor < 4) counter_listMotor++;
      else counter_listMotor = 0;
    }
    if (flag_btn == "TBL_MODE_TUNING_PID") {
      if (counter_tuningPID < 6) counter_tuningPID++;
      else counter_tuningPID = 0;
    }
    if (flag_btn == "TBL_MODE_TUNING_PWM_INPUT") {
      if (counter_tuningPWM_INPUT < 2) counter_tuningPWM_INPUT++;
      else counter_tuningPWM_INPUT = 0;
    }
    if (flag_btn == "TBL_MODE_TUNING_RPM_INPUT") {
      if (counter_tuningRPM_INPUT < 2) counter_tuningRPM_INPUT++;
      else counter_tuningRPM_INPUT = 0;
    }
    if (flag_btn == "TBL_MODE_TUNING_PID_INPUT") {
      if (counter_tuningPID_INPUT < 2) counter_tuningPID_INPUT++;
      else counter_tuningPID_INPUT = 0;
    }
  }

  if (btn2 == 0) {
    while (btn2 == 0) btn2 = digitalRead(TombolMenu[1]);
    lcd.clear();

    if (flag_btn == "TBL_MODE_TUNING_RPM_INPUT" && counter_tuningRPM_INPUT == 0) {
      settingRPM_INPUT[counter_listMotor] -= 10;
    }
    if (flag_btn == "TBL_MODE_TUNING_PWM_INPUT" && counter_tuningPWM_INPUT == 0) {
      settingPWM_INPUT[counter_listMotor] -= 10;
    }
    if (flag_btn == "TBL_MODE_TUNING_PID_INPUT" && counter_tuningPID_INPUT == 0) {
      settingPID_INPUT[counter_listMotor][counter_tuningPID] -= 0.5;
    }
    if (flag_btn == "TBL_MOTOR_START_INPUT" && counter_StartRobot_INPUT == 0) {
      settingStart_INPUT[counter_StartRobot] -= 0.5;
    }


  }
  if (btn3 == 0) {
    while (btn3 == 0) btn3 = digitalRead(TombolMenu[2]);
    lcd.clear();

    if (flag_btn == "TBL_MODE_TUNING_RPM_INPUT" && counter_tuningRPM_INPUT == 0) {
      settingRPM_INPUT[counter_listMotor] += 50;
    }
    if (flag_btn == "TBL_MODE_TUNING_PWM_INPUT" && counter_tuningPWM_INPUT == 0) {
      settingPWM_INPUT[counter_listMotor] += 50;
    }
    if (flag_btn == "TBL_MODE_TUNING_PID_INPUT" && counter_tuningPID_INPUT == 0) {
      settingPID_INPUT[counter_listMotor][counter_tuningPID] += 0.5;
    }
    if (flag_btn == "TBL_MOTOR_START_INPUT" && counter_StartRobot_INPUT == 0) {
      settingStart_INPUT[counter_StartRobot] += 0.5;
    }

  }

  // BUTTON 4
  if (btn4 == 0) {
    while (btn4 == 0) btn4 = digitalRead(TombolMenu[3]);
    lcd.clear();

    //  INTERFACE CONTROLLER
    if (flag_btn == "TBL_MAIN" && counter_mainBtn == 0) {
      flag_btn = "TBL_MODE_ROBOT";
    }
    else if (flag_btn == "TBL_MAIN" && counter_mainBtn == 1) {
      flag_btn = "TBL_INFO_MOTOR";
    }


    else if (flag_btn == "TBL_MODE_ROBOT" && counter_modeRobotBtn == 0) {
      flag_btn = "TBL_MOTOR_START";
    }


    else if (flag_btn == "TBL_MODE_ROBOT" && counter_modeRobotBtn == 0) {
      flag_btn = "TBL_MOTOR_START";
    }


    else if (flag_btn == "TBL_MODE_ROBOT" && counter_modeRobotBtn == 1) {
      MODE_ROBOT_NOW = ROBOT_STOP;
    }
    else if ( flag_btn == "TBL_INFO_MOTOR") {
      flag_btn = "TBL_MAIN";
    }
    else if (flag_btn == "TBL_MODE_ROBOT" &&  (counter_modeRobotBtn == 2 || counter_modeRobotBtn == 3 || counter_modeRobotBtn == 4)  ) {
      flag_btn = "TBL_LIST_MOTOR";
      if (counter_modeRobotBtn == 1) {
        MODE_ROBOT_NOW = ROBOT_STOP;
      }
      if (counter_modeRobotBtn == 2) {
        MODE_ROBOT_NOW = DEBUG_ROBOT_RPM;
        Serial.println("oe rpm");
      }
      if (counter_modeRobotBtn == 4) {
        MODE_ROBOT_NOW = DEBUG_ROBOT_PWM;
        Serial.println("oe pwm");
      }
    }
    else if (flag_btn == "TBL_LIST_MOTOR" && counter_modeRobotBtn == 2 && (counter_listMotor == 0 || counter_listMotor == 1 || counter_listMotor == 2 || counter_listMotor == 3)  ) {
      flag_btn = "TBL_MODE_TUNING_RPM_INPUT";
    }
    else if (flag_btn == "TBL_LIST_MOTOR" && counter_modeRobotBtn == 3 && (counter_listMotor == 0 || counter_listMotor == 1 || counter_listMotor == 2 || counter_listMotor == 3)  ) {
      flag_btn = "TBL_MODE_TUNING_PID";
    }
    else if (flag_btn == "TBL_LIST_MOTOR" && counter_modeRobotBtn == 4 && (counter_listMotor == 0 || counter_listMotor == 1 || counter_listMotor == 2 || counter_listMotor == 3)  ) {
      flag_btn = "TBL_MODE_TUNING_PWM_INPUT";

    }

    else if (flag_btn == "TBL_MOTOR_START" && counter_StartRobot < 3) {
      flag_btn = "TBL_MOTOR_START_INPUT";
    }

    else if (flag_btn == "TBL_MODE_TUNING_PID" && counter_tuningPID < 6 ) {
      flag_btn = "TBL_MODE_TUNING_PID_INPUT";
    }

    // INTERFACE CONTROLLER - EXIT BUTTON
    if (flag_btn == "TBL_MODE_ROBOT" ) {
      if (counter_modeRobotBtn == 5) {
        counter_modeRobotBtn = 0;
        flag_btn = "TBL_MAIN";
      }
    }

    if (flag_btn == "TBL_LIST_MOTOR" ) {
      if (counter_listMotor == 4) {
        counter_listMotor = 0;
        flag_btn = "TBL_MODE_ROBOT";
      }
    }
    if (flag_btn == "TBL_MOTOR_START" ) {
      if (counter_StartRobot == 3) {
        counter_StartRobot = 0;
        flag_btn = "TBL_MODE_ROBOT";
      }
    }
    if (flag_btn == "TBL_MOTOR_START_INPUT" ) {
      if (counter_StartRobot_INPUT == 2) {
        counter_StartRobot_INPUT = 0;
        flag_btn = "TBL_MOTOR_START";
      }
    }
    if (flag_btn == "TBL_MODE_TUNING_PID" ) {
      if (counter_tuningPID == 6) {
        counter_tuningPID = 0;
        flag_btn = "TBL_LIST_MOTOR";
      }
    }

    if (flag_btn == "TBL_MODE_TUNING_RPM_INPUT" ) {
      if (counter_tuningRPM_INPUT == 2) {
        counter_tuningRPM_INPUT = 0;
        flag_btn = "TBL_LIST_MOTOR";
      }
    }
    if (flag_btn == "TBL_MODE_TUNING_PID_INPUT" ) {
      if (counter_tuningPID_INPUT == 2) {
        counter_tuningPID_INPUT = 0;
        flag_btn = "TBL_MODE_TUNING_PID";
      }
    }

    if (flag_btn == "TBL_MODE_TUNING_PWM_INPUT" ) {
      if (counter_tuningPWM_INPUT == 2) {
        counter_tuningPWM_INPUT = 0;
        flag_btn = "TBL_LIST_MOTOR";
      }
    }

    // IF OK SET VARIABLE
    if (flag_btn == "TBL_MODE_TUNING_RPM_INPUT" && counter_tuningRPM_INPUT == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SAVING PARAMETER");
      delay(100); lcd.clear();

      settingRPM[counter_listMotor] = settingRPM_INPUT[counter_listMotor];
    }
    if (flag_btn == "TBL_MODE_TUNING_PWM_INPUT" && counter_tuningPWM_INPUT == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SAVING PARAMETER");
      delay(100); lcd.clear();

      settingPWM[counter_listMotor] = settingPWM_INPUT[counter_listMotor];
    }
    if (flag_btn == "TBL_MODE_TUNING_PID_INPUT" && counter_tuningPID_INPUT == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SAVING PARAMETER");
      delay(100); lcd.clear();

      settingPID[counter_listMotor][counter_tuningPID ] =  settingPID_INPUT[counter_listMotor][counter_tuningPID];
    }

    if (flag_btn == "TBL_MOTOR_START_INPUT" && counter_StartRobot_INPUT == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SAVING PARAMETER");
      delay(100); lcd.clear();

      settingStart[counter_StartRobot] = settingStart_INPUT[counter_StartRobot];
    }
  }
}
