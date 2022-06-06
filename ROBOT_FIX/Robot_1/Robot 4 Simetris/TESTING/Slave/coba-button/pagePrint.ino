void tampil() {

  Serial.println(btn1);

  if (flagWelcome == 0) {
    lcd.clear();
    lcdDisplayAtas(1, 1);
    flagWelcome = 1;

    lcdDisplayBawah(countMenuMode, 1);
    countMenuMode++;
  }

  if (btn1 == 0) {
    while (btn1 == 0) btn1 = digitalRead(15);

    if (countMenuMode >= 4) {
      countMenuMode = 0;
    }

    lcdDisplayAtas(1, 1);
    lcdDisplayBawah(countMenuMode, 1);
    countMenuMode++;
  }

  if (btn2 == 0) {
    while (btn2 == 0) btn2 = digitalRead(4);
  }

  if (btn3 == 0) {
    while (btn3 == 0) btn3 = digitalRead(18);
  }

  if (btn4 == 0) {
    while (btn4 == 0) btn4 = digitalRead(19);
  }


}

void lcdDisplayAtas(int page, int isClear) {
  if (isClear == 1) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
  }


  if (page == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Main Menu Robot:");
  }

  if (page == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Starting Robot");
  }

  if (page == 3) {
    lcd.setCursor(0, 0);
    lcd.print("PWM MTR");
  }
  if (page == 4) {
    lcd.setCursor(0, 0);
    lcd.print("PID TUNING:");
  }
}

void lcdDisplayBawah(int page, int isClear) {
  if (isClear == 1) {
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
  if (page == 0) {
    lcd.setCursor(0, 1);
    lcd.print("> START");
  }
  else if (page == 1) {
    lcd.setCursor(0, 1);
    lcd.print("> INFO PARAM");
  }
  else if (page == 2) {
    lcd.setCursor(0, 1);
    lcd.print("> TES PWM MOTOR");
  }
  else if (page == 3) {
    lcd.setCursor(0, 1);
    lcd.print("> TUNING PID");
  }

  if (page == 10) {
    lcd.setCursor(0,  1);
    lcd.print("-ROBOT BERJALAN-");
  }
  if (page == 11) {
    lcdClearAtas() ;
    lcd.setCursor(0,  1);
    lcd.print("-ROBOT BERHENTI-");
    delay(4000);
    lcdClearBawah();
    lcdClearAtas() ;
    lcd.setCursor(0,  1);
    lcd.print("Kembali Ke Menu.");
    delay(2000);
  }

  if (page == 12) {
    lcd.setCursor(0,  1);
    lcd.print("RPM ");
  }

  // PID
  if (page == 20) {
    lcd.setCursor(0,  1);
    lcd.print("-> MOTOR 1");
  }
  if (page == 21) {
    lcd.setCursor(0,  1);
    lcd.print("-> MOTOR 2");
  }

  if (page == 22) {
    lcd.setCursor(0,  1);
    lcd.print("-> MOTOR 3");
  }

  if (page == 23) {
    lcd.setCursor(0,  1);
    lcd.print("-> MOTOR 4");
  }

  if (page == 25) {
    lcd.setCursor(0,  1);
    lcd.print("--> Sp");
  }
  if (page == 26) {
    lcd.setCursor(0,  1);
    lcd.print("--> Kp");
  }
  if (page == 27) {
    lcd.setCursor(0,  1);
    lcd.print("--> Ki");
  }
  if (page == 28) {
    lcd.setCursor(0,  1);
    lcd.print("--> Kd");
  }
  if (page == 29) {
    lcd.setCursor(0,  1);
    lcd.print("--> Ti");
  }
  if (page == 30) {
    lcd.setCursor(0,  1);
    lcd.print("--> Td");
  }
}

void lcdClearBawah() {
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

void lcdClearAtas() {
  lcd.setCursor(0, 0);
  lcd.print("                ");
}
// PWM
void lcdPrintResponRPM(int motor, int rpm) {
  lcd.setCursor(4,  1);
  lcd.print("MTR " + String(motor) + " : " + String(rpm));
}
void lcdPrintSetPWM(int motor, int PWM) {
  lcd.setCursor(8, 0);
  lcd.print("        ");
  lcd.setCursor(8, 0);
  lcd.print(String(motor) + " : " + String(PWM) + "  ");
}

// PID
void lcdPrintMotorMana(int motor) {
  lcdDisplayBawah(motor, 1);
}

//void tampil() {
////  if (msTimer.fire()) {
////    Serial.println(flagMenuSetPID);
////  }
//  Serial.println(btn1);
//
//  if (flagWelcome == 0) {
//    lcd.clear();
//    lcdDisplayAtas(1, 1);
//    flagWelcome = 1;
//
//    lcdDisplayBawah(countMenuMode, 1);
//    countMenuMode++;
//  }
//
//  if (btn1 == 0) {
//    while (btn1 == 0) {
//      btn1 = digitalRead(15);
//    }
//    if (countMenuMode >= 4) {
//      countMenuMode = 0;
//    }
//
//    lcdDisplayAtas(1, 1);
//    lcdDisplayBawah(countMenuMode, 1);
//    countMenuMode++;
//
//    // Kembali dari menu ditekan
//    if (flagMenu2 == 10) {
//      lcdDisplayAtas(1, 1);
//      lcdDisplayBawah(flagMenu1, 1);
//      countMenuMode = flagMenu1;
//      flagMenu2 = 0;
//    }
//
//    // Kembali dari Menu Set PWM
//    if (flagmenuSetPWM == 1) {
//      countMenuMode = 1;
//      flagMenu1 = 0;
//      flagMenu2 = 12;
//      flagmenuSetPWM = 0;
//
//      lcdDisplayAtas(1, 1);
//      lcdDisplayBawah(flagMenu1, 1);
//
//    }
//    if (flagMenuSetPID == 1) {
//      countMenuMode = 2;
//      flagMenu1 = 0;
//      flagMenu2 = 13;
//      flagMenuSetPID = 0;
//
//      lcdDisplayAtas(1, 1);
//      lcdDisplayBawah(flagMenu1, 1);
//    }
//    if (flagMenuSetPID == 2) {
//      countMenuMode = 2;
//      flagMenu1 = 3;
//      flagMenu2 = 13;
//
//      flagMenuSetPID = 1;
//      lcdDisplayAtas(1, 1);
//      lcdDisplayBawah(menuSetPID + 20, 1);
//    }
//  }
//
//  if (btn2 == 0) {
//    while (btn2 == 0) {
//      btn2 = digitalRead(4);
//    }
//    if (flagMenuSetPID != 2) flagMenu1 = countMenuMode - 1;
//    // Tekan Awal (Menu) > Start
//    //    Serial.println("flagMenu1 : " + String(flagMenu1));
//    if (flagMenu1 == 0) {
//      flagMenu2 = 10;
//      lcdDisplayAtas(2, 1);
//      lcdDisplayBawah(flagMenu2, 1);
//
//    }
//    // Tekan Awal (Menu) => Start => STOP
//    else if (flagMenu1 == 1) {
//      flagMenu2 = 11;
//      lcdDisplayAtas(2, 1);
//      lcdDisplayBawah(flagMenu2, 1);
//      //
//      lcdDisplayAtas(1, 1);
//      lcdDisplayBawah(flagMenu1, 1);
//      countMenuMode = flagMenu1;
//      flagMenu2 = 0;
//    }
//    // Tekan Awal (Menu) => TES PWM
//    else if (flagMenu1 == 2) {
//      flagMenu2 = 12;
//      lcdDisplayAtas(3, 1);
//      lcdDisplayBawah(flagMenu2, 1);
//      flagmenuSetPWM = 1;
//
//    }
//    // Tekan Awal (Menu) => Tuning PID
//    else if (flagMenu1 == 3) {
//      flagMenu2 = 20;
//      lcdDisplayAtas(4, 1);
//      lcdDisplayBawah(flagMenu2, 1);
//      flagMenuSetPID = 1;
//      flagMenu1 = 30;
//    }
//
//    // SettingPWM MENU
//
//    if (flagmenuSetPWM == 1) {
//      menuSetPWM++;
//      if (menuSetPWM > 3) {
//        menuSetPWM = 0;
//      }
//    }
//    // Setting PID MENU
//    if (flagMenuSetPID == 1) {
//      menuSetPID--;
//      if (menuSetPID < 0) {
//        menuSetPID = 0;
//      }
//    }
//    if (flagMenuSetPID == 2) {
//      menuSetPID--;
//      if (menuSetPID < 0) {
//        menuSetPID = 0;
//      }
//    }
//  }
//
//  if (btn3 == 0) {
//    while (btn3 == 0) {
//      btn3 = digitalRead(18);
//    }
//
//    // Jika Robot Jalan Ditekan Stop
//    if (flagMenu2 == 10) {
//      flagMenu2 = 11;
//      lcdDisplayAtas(2, 1);
//      lcdDisplayBawah(flagMenu2, 1);
//      //
//      lcdDisplayAtas(1, 1);
//      lcdDisplayBawah(flagMenu1, 1);
//      countMenuMode = flagMenu1;
//      flagMenu2 = 0;
//    }
//    // Nilai PWM Kurang pada menu PWM
//    if (flagmenuSetPWM == 1) {
//      flagKrgPWM = 1;
//    }
//
//    if (flagMenuSetPID == 1) {
//      menuSetPID++;
//      if (menuSetPID > 3) {
//        menuSetPID = 3;
//      }
//    }
//    if (flagMenuSetPID == 2) {
//      menuSetPID++;
//      if (menuSetPID > 5) {
//        menuSetPID = 5;
//      }
//    }
//  }
//
//  if (btn4 == 0) {
//    while (btn4 == 0) {
//      btn4 = digitalRead(19);
//    }
//    // Nilai PWM Kurang pada menu PWM
//    if (flagmenuSetPWM == 1) {
//      flagTmbPWM = 1;
//    }
//
//    if (flagMenuSetPID == 1) {
//      Serial.println("TEKAN");
//      flagMenuSetPID = 2;
//    }
//  }
//
//  // Controller Menu SET PWM
//  if (flagmenuSetPWM == 1) {
//    if (flagTmbPWM == 1) {
//      valPWM_MOTOR[menuSetPWM] += 50;
//      if ( valPWM_MOTOR[menuSetPWM] > 1023) {
//        valPWM_MOTOR[menuSetPWM] = 1023;
//      }
//      flagTmbPWM = 0;
//    }
//    if (flagKrgPWM == 1) {
//      valPWM_MOTOR[menuSetPWM] -= 50;
//      if ( valPWM_MOTOR[menuSetPWM] < 0) {
//        valPWM_MOTOR[menuSetPWM] = 0;
//      }
//      flagKrgPWM = 0;
//    }
//
//    if (msTimer1.fire()) {
//      lcdPrintSetPWM(menuSetPWM + 1, valPWM_MOTOR[menuSetPWM]);
//      lcdPrintResponRPM(menuSetPWM + 1, random(0, 200));
//    }
//  }
//
//  if (flagMenuSetPID > 0) {
//    if (flagMenuSetPID == 1) { // Motor MANA?
//      if (msTimer1.fire()) lcdPrintMotorMana( menuSetPID + 20);
//    }
//    if (flagMenuSetPID == 2) { // SET PARAMETER PID Motor MANA?
//      if (msTimer2.fire()) lcdPrintMotorMana( menuSetPID + 25);
//    }
//  }
//}
