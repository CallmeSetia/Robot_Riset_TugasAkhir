// INTERFACE
char TampilanAtas[][20] = {"Main Menu:", "Starting Robot"};
char TampilanBawahMain[][20] = {"> MODE ROBOT", "> INFO MOTOR"};

char TampilanBawahModeRobot[][20] = {"> ROBOT START",  "> ROBOT STOP", "> SET RPM", "> SET PID", "> SET PWM", "> EXIT" };
char TampilanBawahListMotor[][20] = {"> Motor 1",  "> Motor 2", "> Motor 3", "> Motor 4", "> EXIT"};

char TampilanAtasModeStart[][20] = {"Start Robot:", "  "};
char TampilanBawahModeStart[][20] = {"> Set X", "> Set Y", "> Set Theta", "> EXIT"};
char TampilanAtasTuningModeStart[][20] = {"Set X Robot: ", "Set Y Robot: ", "Set Theta Rbt:", "> EXIT"};

char TampilanAtasModeTuningPID[][20] = {"Tunning PID M1:",  "Tunning PID M2:", "Tunning PID M3:", "Tunning PID M4:", "  "};
char TampilanAtasModeTuningPWM[][20] = {"Tunning PWM M1:",  "Tunning PWM M2:", "Tunning PWM M3:", "Tunning PWM M4:", "  "};
char TampilanAtasModeTuningRPM[][20] = {"Tunning RPM M1:",  "Tunning RPM M2:", "Tunning RPM M3:", "Tunning RPM M4:", "  "};
char TampilanBawahModeTuningPID[][20] = {"> Set KP",  "> Set KI", "> Set KD", "> Set Ts", "> Set Ti", "> Set Td", "> EXIT" };
char TampilanBawahTuning[][20] = {":= ", "> OK", "> EXIT", " " };
char TampilanAtasTuningPID[][20] = {"Set Kp", "Set Ki", "Set Kd", "Set Ts", "Set Ti", "Set Td", "   "};


String flag_btn = "TBL_MAIN";
int counter_mainBtn = 0, counter_modeRobotBtn = 0, counter_listMotor = 0, counter_tuningPID = 0, counter_StartRobot = 0,
    counter_tuningPWM_INPUT = 0, counter_tuningPID_INPUT = 0, counter_tuningRPM_INPUT = 0, counter_StartRobot_INPUT = 0;

void lcdInit() {
  lcd.begin();
  lcd.backlight();
}

void introDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("-=TUGAS AKHIR=-");
  lcd.setCursor(0, 1);
  lcd.print(" -=2021/2022=-");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1.Wiji Ningsih");
  lcd.setCursor(0, 1);
  lcd.print("2.Dwi Setia F");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("-=SYSTEM INIT=-");
  lcd.setCursor(0, 1);
  lcd.print("-=ROBOT MULAI=-");

  delay(2000);
}

void interfaceMain() {
  if (flag_btn == "TBL_MAIN") {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtas[0]);
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahMain[counter_mainBtn]);
  }

  else if (flag_btn == "TBL_MOTOR_START") {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtasModeStart[0]);
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahModeStart[counter_StartRobot]);
    
  }

   else if (flag_btn == "TBL_MOTOR_START_INPUT" ) {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtasTuningModeStart[counter_StartRobot] );
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahTuning[counter_StartRobot_INPUT]);

    if (counter_StartRobot_INPUT == 0) {
      lcd.setCursor(3, 1);
      lcd.print(settingStart_INPUT[counter_StartRobot]);
    }
  }


  
  else if (flag_btn == "TBL_INFO_MOTOR") {
    if (timerNgeprint2.fire()) {
      char layar1[17];
      char layar2[17];
      sprintf(layar1, "M1: %03d M3: %03d", rpm1, rpm3);
      sprintf(layar2, "M2: %03d M4: %03d", rpm2, rpm4);
      lcd.setCursor(0, 0);
      lcd.print(layar1);
      lcd.setCursor(0, 1);
      lcd.print(layar2);
      
      memset(layar1, 0, 17);
      memset(layar2, 0, 17);

    }

  }

  else if (flag_btn == "TBL_MODE_ROBOT" ) {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtas[0]);
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahModeRobot[counter_modeRobotBtn]);
  }
  else if (flag_btn == "TBL_LIST_MOTOR" ) {
    lcd.setCursor(0, 0);
    if (counter_modeRobotBtn == 2) { // Tune RPM
      lcd.print(TampilanAtasModeTuningRPM[counter_listMotor] );

    }
    else if (counter_modeRobotBtn == 3) { // Tune PID
      lcd.print(TampilanAtasModeTuningPID[counter_listMotor]);
    }
    else if (counter_modeRobotBtn == 4) { // Tune PWM
      lcd.print(TampilanAtasModeTuningPWM[counter_listMotor]);
      //lcd.print(String(counter_mainBtn) + String(counter_modeRobotBtn) + String(counter_listMotor) + String(counter_tuningPWM_INPUT));

    }

    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahListMotor[counter_listMotor]);
  }
  else if (flag_btn == "TBL_MODE_TUNING_PID") {
    //Serial.println("HAI" + flag_btn);
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtasModeTuningPID[counter_listMotor]);
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahModeTuningPID[counter_tuningPID]);
  }

  else if (flag_btn == "TBL_MODE_TUNING_PID_INPUT" ) {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtasTuningPID[counter_tuningPID] );
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahTuning[counter_tuningPID_INPUT]);

    if (counter_tuningPID_INPUT == 0) {
      lcd.setCursor(3, 1);
      lcd.print(settingPID_INPUT[counter_listMotor][counter_tuningPID]);
    }
  }

  else if (flag_btn == "TBL_MODE_TUNING_PWM_INPUT") {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtasModeTuningPWM[counter_listMotor]);
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahTuning[counter_tuningPWM_INPUT]);

    if (counter_tuningPWM_INPUT == 0) {
      lcd.setCursor(3, 1);
      lcd.print(settingPWM_INPUT[counter_listMotor]);
    }
  }
  else if (flag_btn == "TBL_MODE_TUNING_RPM_INPUT") {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtasModeTuningRPM[counter_listMotor]);
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahTuning[counter_tuningRPM_INPUT]);

    if (counter_tuningRPM_INPUT == 0) {
      lcd.setCursor(3, 1);
      lcd.print(settingRPM_INPUT[counter_listMotor]);
    }
  }
}
