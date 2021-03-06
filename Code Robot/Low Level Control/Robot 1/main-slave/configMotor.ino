
// Setting PWM properties
const int freq = 15000;
const int resolution = 10;

int pwmOut1;
int pwmOut2;

int enablePID;
int arahPutarMotor1;
int arahPutarMotor2;


// TIMER PID KALKULASI
void IRAM_ATTR TIMER_PID_M1(void) {
  pwmOut1 =  motor1_PID.kalkulasi(rpm1);
}
void IRAM_ATTR TIMER_PID_M2(void) {
  pwmOut2 =  motor2_PID.kalkulasi(rpm2);
}

void IRAM_ATTR TimerHandler1(void) {
  encCnt1 = encoder1.getCount();
  deltaCount1 = encCnt1 - lastCount1;
  rpm1 = ((deltaCount1) * 600) / 745;
  lastCount1 = encCnt1;
}

void IRAM_ATTR TimerHandler2(void) {
  encCnt2 = encoder2.getCount();
  deltaCount2 = encCnt2 - lastCount2;
  rpm2 = ((deltaCount2) * 600) / 745;
  lastCount2 = encCnt2;
}



void configMotor() {


  // TIMER BACA ENCODER
  //
  TimerPID1.attachInterruptInterval(100 * 1000, TIMER_PID_M1);
  TimerPID2.attachInterruptInterval(100 * 1000, TIMER_PID_M2);
  
  ITimer1.attachInterruptInterval(100 * 1000, TimerHandler1);
  ITimer2.attachInterruptInterval(100 * 1000, TimerHandler2);

  ESP32Encoder::useInternalWeakPullResistors = UP;   // Attache pins for use as encoder pins

//  msTimer.begin(5);
//  timerNgeprint1.begin(5);
  timerNgeprint2.begin(200);
  timerNgeprint3.begin(5);
//  timerNgeprint4.begin(5);
  // ---- MOTOR 1
  motor1.pin(32, 33);
  motor1.enc(36, 39);
  motor1.pwm_ch(0, 1);

  // --- MOTOR PID
  motor1_PID.mulai();

  // --- MOTOR 1 PWM
  ledcSetup(motor1.pwm_ch1, freq, resolution);
  ledcSetup(motor1.pwm_ch2, freq, resolution);

  ledcAttachPin(motor1.pin1, motor1.pwm_ch1);
  ledcAttachPin(motor1.pin2, motor1.pwm_ch2);
  // -------------------------------------

  // ---- MOTOR 2
  motor2.pin(25, 26);
  motor2.enc(35, 34);
  motor2.pwm_ch(2, 3);

  // --- MOTOR 2 PID
  motor2_PID.mulai();
  
  // --- MOTOR 4 PWM
  ledcSetup(motor2.pwm_ch1, freq, resolution);
  ledcSetup(motor2.pwm_ch2, freq, resolution);

  ledcAttachPin(motor2.pin1, motor2.pwm_ch1);
  ledcAttachPin(motor2.pin2, motor2.pwm_ch2);
  // -------------------------------------

  encoder1.attachHalfQuad(motor1.enc1, motor1.enc2); //encoderA, encoderB
  encoder2.attachHalfQuad(motor2.enc1, motor2.enc2); //encoderA, encoderB

  // count awal - start
  encoder1.setCount(0);
  encoder2.setCount(0);
}
