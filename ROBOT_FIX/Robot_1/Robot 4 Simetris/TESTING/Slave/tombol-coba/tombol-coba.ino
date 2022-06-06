#include <ros.h>
#include <robot_riset/TombolMenu.h>

#define ROBOT_ID 1


int TombolMenu[4] = {15, 4, 18, 19};

const char* TampilanAtas[][20] = {
  {"Main Menu Robot:", "Starting Robot", "PWM MTR", "PID TUNING:"}
};

const char* TampilanBawah[][20] = {
  {"> START", "> INFO PARAM", "> TES PWM MOTOR", "> TUNING PID"},
};


// ======= ROS Konfigurasi ====
// Ros Node
ros::NodeHandle Robot1;

// ROS MSG
robot_riset::TombolMenu tombolMenu; // Msg Tombol


// ROS PUB
ros::Publisher tombolMenu_Pub("robot_riset/TombolMenu", &tombolMenu);


void setup() {
  init_TombolMenu(TombolMenu);
  Serial.begin(115200);

  
  Robot1.initNode();
  Robot1.getHardware()->setBaud(115200);
  Robot1.advertise(tombolMenu_Pub);

  Robot1.loginfo("INISIALISASI AMAN");

}

void loop() {
  Robot1.loginfo("INISIALISASI AMAN");

  int btn1 = bacaTombol(TombolMenu[0]);
  int btn2 = bacaTombol(TombolMenu[1]);
  int btn3 = bacaTombol(TombolMenu[2]);
  int btn4 = bacaTombol(TombolMenu[3]);

  if (btn1 == 0) {
    while (btn1 == 0) btn1 = digitalRead(15);
    Robot1.loginfo("TOMBOL 1 TEKAN");
  }

  if (btn2 == 0) {
    while (btn2 == 0) btn2 = digitalRead(4);
    Robot1.loginfo("TOMBOL 2 TEKAN");
  }

  if (btn3 == 0) {
    while (btn3 == 0) btn3 = digitalRead(18);
    Robot1.loginfo("TOMBOL 3 TEKAN");
  }

  if (btn4 == 0) {
    while (btn4 == 0) btn4 = digitalRead(19);
    Robot1.loginfo("TOMBOL 4 TEKAN");
  }

  tombolMenu.state_tombol1 = btn1;
  tombolMenu.state_tombol2 = btn2;
  tombolMenu.state_tombol3 = btn3;
  tombolMenu.state_tombol4 = btn4;

  tombolMenu_Pub.publish(&tombolMenu);
  Robot1.spinOnce();
  delay(10);
}
