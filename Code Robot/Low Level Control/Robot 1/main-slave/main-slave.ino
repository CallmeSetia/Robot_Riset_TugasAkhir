#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#include "SerialTransfer.h"
#include <ArduinoJson.h>
#include <ESP32Encoder.h>
#include "ESP32TimerInterrupt.h"

#include "PID_Kontrol.h"
#include "Motor.h"
#include "FireTimer.h"

SerialTransfer keMaster;
FireTimer msTimer;
FireTimer timerNgeprint1, timerNgeprint2, timerNgeprint3, timerNgeprint4 ;

Motor motor1;
Motor motor2;

PID_Kontrol motor1_PID(1, 0, 0, 80, -248, 248, 0, 1023);
PID_Kontrol motor2_PID(1, 0, 0, 80, -248, 248, 0, 1023);

ESP32Timer ITimer1(0);
ESP32Timer ITimer2(1);
ESP32Timer TimerPID1(2);
ESP32Timer TimerPID2(3);

ESP32Encoder encoder1;
ESP32Encoder encoder2;

void setup() {
  lcdInit();
  tombol_init();

  Serial.begin(9600);    // USB
  Serial2.begin(115200); // ESP Master

  keMasterConfig();
  configMotor();

  RTOS_Init(); // Dual Core ESP32
  lcd.clear();
}

void loop() {
  bacaTombol();    // Read BTN
  tombolHandler(); // TOMBOL HANDLER
  interfaceMain(); // Display LCD HANDLER
  control_motor(); // CTRL MOTOR
}


void SlaveMasterComunicationTask( void * pvParameters ) {
  for (;;) {
    keMaster.tick();           // Baca Dari Master
    kirimInformasiKeMaster();  // Kirim ke Master

    vTaskDelay( 20 / portTICK_PERIOD_MS ); // vTask Delay untuk Pindah Proses ke Core 0
  }
}
