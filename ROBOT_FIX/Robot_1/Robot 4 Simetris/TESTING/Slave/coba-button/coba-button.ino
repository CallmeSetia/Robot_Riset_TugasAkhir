#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "FireTimer.h"

FireTimer msTimer, msTimer1, msTimer2, msTimer3, timerTampil;
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

int countMenuMode = 0;
int flagWelcome = 0;
int flagMenu1 = 0;
int flagMenu2 = 0;
int flagMenu3 = 0;
int flagMenu4 = 0;

int flagmenuSetPWM = 0;
int menuSetPWM = 0;
int valPWM_MOTOR[4] = {0, 0, 0, 0};
int flagTmbPWM = 0;
int flagKrgPWM = 0;

int flagMenuSetPID = 0;
int menuSetPID = 0;

int valPID_Motor[6][4] = {
  {0, 0, 0, 0}, // Sp
  {0, 0, 0, 0}, // KP
  {0, 0, 0, 0}, // KI
  {0, 0, 0, 0}, // KD
  {0, 0, 0, 0}, // Ti
  {0, 0, 0, 0}, // Td
};


int btn1;
int btn2;
int btn3;
int btn4 ;
void setup()
{
  Serial.begin(9600);
  msTimer.begin(1000);
  msTimer1.begin(200);
  msTimer2.begin(200);
  msTimer3.begin(200);
timerTampil.begin(200);

  lcd.begin();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistem Memulai..");
  delay(1000);
  Serial.println("INIT");

  // GPIO - SETUP
  pinMode(15, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  // initialize the LCD

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tugas Akhir 2022");
  lcd.setCursor(0, 1);
  lcd.print("  -=POLINEMA=-  ");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nama 1");
  lcd.setCursor(0, 1);
  lcd.print("Nama 2");
  delay(1000);
}

void loop()
{
  btn1 = digitalRead(15);
  btn2 = digitalRead(4);
  btn3 = digitalRead(18);
  btn4 = digitalRead(19);
  
  if ( timerTampil.fire()) tampil();
}
