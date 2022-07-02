#include <ESP32Encoder.h>
#include "ESP32TimerInterrupt.h"

ESP32Encoder encoder;
ESP32Encoder encoder2;

ESP32Timer ITimer1(0);
ESP32Timer ITimer2(1);


//Motor 1
volatile int lastCount1 = 0;
volatile int deltaCount1 = 0;
volatile int rpm1 = 0;
volatile int pwm1 = 0;

volatile int encCnt1;

//Motor 2
volatile int lastCount2 = 0;
volatile int deltaCount2 = 0;
volatile int rpm2 = 0;
volatile int pwm2 = 0;
volatile int encCnt2;

int rpm = 0;

// timer and flag for example, not needed for encoders
unsigned long encoder2lastToggled;
bool encoder2Paused = false;

//Motor A
int motorAPin1 = 32; //LPWM
int motorAPin2 = 33; //RPWM
//int enableAPin = 14;
//Motor B
int motorBPin1 = 25; //LPWM
int motorBPin2 = 26; //RPWM
//int enableBPin = 32;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;
const int resolution = 10;
int PWM1 = 500;
int PWM2 = 500;

void IRAM_ATTR TimerHandler2(void) {
  encCnt2 = encoder2.getCount();
}

void IRAM_ATTR TimerHandler1(void) {
  encCnt1 = encoder1.getCount();
}

void motorAmaju()
{
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
}

void motorBmaju()
{
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
}

void setup() {

  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  //pinMode(enableAPin, OUTPUT);

  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  //pinMode(enableBPin, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcSetup(pwmChannel3, freq, resolution);
  ledcSetup(pwmChannel4, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motorAPin1, pwmChannel1);
  ledcAttachPin(motorAPin2, pwmChannel2);
  ledcAttachPin(motorBPin1, pwmChannel3);
  ledcAttachPin(motorBPin2, pwmChannel4);


  Serial.begin(115200);
  Serial.print("Testing DC Motor...");

  ITimer1.attachInterruptInterval(100 * 1000, TimerHandler1);
  ITimer2.attachInterruptInterval(100 * 1000, TimerHandler2);

  // Enable the weak pull down resistors

  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = UP;

  // Attache pins for use as encoder pins
  encoder.attachHalfQuad(39, 36); //encoderA, encoderB
  // Attache pins for use as encoder pins
  encoder2.attachHalfQuad(34, 35); //encoderA, encoderB

  // set starting count value after attaching
  encoder.setCount(0);
  encoder2.setCount(0);

  // clear the encoder's raw count and set the tracked count to zero
  //encoder2.clearCount();
  // Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));
  // set the lastToggle
  encoder2lastToggled = millis();
}

void loop() {

  motorAmaju();
  motorBmaju();
  ledcWrite(pwmChannel1, PWM1);
  ledcWrite(pwmChannel2, PWM2);
  ledcWrite(pwmChannel3, PWM1);
  ledcWrite(pwmChannel4, PWM2);

  // Loop and read the count
  Serial.println("Encoder count = " + String((int32_t)encCnt1) + " " + String((int32_t)encCnt2));

  //  rpm = (float)(encoder.getCount() * 60 / 2984);
  //  Serial.print(rpm);
  //  Serial.println(" RPM");

  // every 5 seconds toggle encoder 2
  /* if (millis() - encoder2lastToggled >= 5000) {
     if(encoder2Paused) {
       Serial.println("Resuming Encoder 2");
       encoder2.resumeCount();
     } else {
       Serial.println("Paused Encoder 2");
       encoder2.pauseCount();
     }

     encoder2Paused = !encoder2Paused;
     encoder2lastToggled = millis();
    }*/
}
