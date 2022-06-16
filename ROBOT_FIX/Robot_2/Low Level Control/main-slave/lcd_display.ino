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
    lcd.print(TampilanAtas[0] + String(counter_mainBtn));
    lcd.setCursor(0, 1);
    lcd.print(TampilanBawahMain[counter_mainBtn]);
  }
  else if (flag_btn == "TBL_INFO_MOTOR") {
    if (timerNgeprint2.fire()) {
      lcd.setCursor(0, 0);
      lcd.print("M1:" + String(rpm1) + " M3:" + String(debug_RPM_Motor[3]) );
      lcd.setCursor(0, 1);
      lcd.print("M2:" + String(rpm2) + " M4:" + String(debug_RPM_Motor[4]) );
    }

  }

  else if (flag_btn == "TBL_MODE_ROBOT" ) {
    lcd.setCursor(0, 0);
    lcd.print(TampilanAtas[0] + String(counter_mainBtn) + String(counter_modeRobotBtn));
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
