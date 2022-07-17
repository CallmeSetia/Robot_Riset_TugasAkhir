




void ROS_INIT() {
  Robot.initNode();
  while (!Robot.connected()) {
    Robot.spinOnce();
  }



  Robot.advertise(MotorRaw_RPM);
  Robot.advertise(MotorRaw_PWM);
  Robot.advertise(ModeRobot_Pub);

  Robot.advertise(ParamPWM_Motor_Pub);
  Robot.advertise(ParamRPM_Motor_Pub);

  Robot.advertise(IMU_Accl_Pub);
  Robot.advertise(IMU_Gyro_Pub);
  Robot.advertise(IMU_Quat_Pub);
  Robot.advertise(IMU_Euler_Pub);
  Robot.advertise(IMU_Time_Pub);

  Robot.subscribe(MotorCommand_Sub);
}

void ROS_Publish() {
   // ROS PUBLISH
  modeRobot_Msg.data = MODE_ROBOT_NOW.c_str();
  ModeRobot_Pub.publish(&modeRobot_Msg);



  mpu9250_Accl_ros.x = aX;
  mpu9250_Accl_ros.y = aY;
  mpu9250_Accl_ros.z = aZ;

  mpu9250_Gyro_ros.x = gX;
  mpu9250_Gyro_ros.y = gY;
  mpu9250_Gyro_ros.z = gZ;

  mpu9250_Quat_ros.x = q0;
  mpu9250_Quat_ros.y = q1;
  mpu9250_Quat_ros.z = q2;
  mpu9250_Quat_ros.w = q3;

  mpu9250_Euler_ros.x = Yaw;
  mpu9250_Euler_ros.y = Pitch;
  mpu9250_Euler_ros.z = Roll;


  mpu9250_IMUTime_ros.data = IMU_Time;

  IMU_Accl_Pub.publish(&mpu9250_Accl_ros);
  IMU_Gyro_Pub.publish(&mpu9250_Gyro_ros);
  IMU_Quat_Pub.publish(&mpu9250_Quat_ros);
  IMU_Euler_Pub.publish(&mpu9250_Euler_ros);
  IMU_Time_Pub.publish(&mpu9250_IMUTime_ros);


  MotorEnc1.x = rpm1;
  MotorEnc1.y = rpm2;
  MotorEnc1.z = rpm3;
  MotorEnc1.w = rpm4;

  MotorEnc2.x = debug_PWM_Motor[0];
  MotorEnc2.y = debug_PWM_Motor[1];
  MotorEnc2.z = debug_PWM_Motor[2];
  MotorEnc2.w = debug_PWM_Motor[3];


  ParamPWM_Motor_Pub.publish(&ParamPWM_Motor);
  ParamRPM_Motor_Pub.publish(&ParamRPM_Motor);
  MotorRaw_RPM.publish(&MotorEnc1);
  MotorRaw_PWM.publish(&MotorEnc2);

  Robot.spinOnce();
}


void callback_MotorCommand(const robot_riset::EncoderMotor &Command) {
  String ModeRobot = parseModeRobot(MODE_ROBOT_NOW.toInt());
  if (Command.cmd == 3) {
    
    settingPWM[0] = 0;
    settingPWM[1] = 0;
    settingPWM[2] = 0;
    settingPWM[3] = 0;

    settingRPM[0] = Command.rpm.x;
    settingRPM[1] = Command.rpm.y;
    settingRPM[2] = Command.rpm.z;
    settingRPM[3] = Command.rpm.w;
  }
  else if (Command.cmd == 4) {
    settingPWM[0] = Command.pwm.x;
    settingPWM[1] = Command.pwm.y;
    settingPWM[2] = Command.pwm.z;
    settingPWM[3] = Command.pwm.w;

    settingRPM[0] = 0;
    settingRPM[1] = 0;
    settingRPM[2] = 0;
    settingRPM[3] = 0;
  }
  else if (Command.cmd == 1 || ModeRobot ==  ROBOT_RUN) {
    settingPWM[0] = Command.pwm.x;
    settingPWM[1] = Command.pwm.y;
    settingPWM[2] = Command.pwm.z;
    settingPWM[3] = Command.pwm.w;

    settingRPM[0] = Command.rpm.x;
    settingRPM[1] = Command.rpm.y;
    settingRPM[2] = Command.rpm.z;
    settingRPM[3] = Command.rpm.w;
  }
}

//void callback_MotorRPM (const geometry_msgs::Quaternion  &motor) {
//  String ModeRobot = parseModeRobot(MODE_ROBOT_NOW.toInt());
//  if (ModeRobot == DEBUG_ROS_RPM) {
//    settingRPM[0] = motor.x;
//    settingRPM[1] = motor.y;
//    settingRPM[2] = motor.z;
//    settingRPM[3] = motor.w;
//
////    cmd@rpm1@rpm2@rpm3@rpm4@pwm1@pwm2@pwm3@pwm4 // pwm | rpm @ roda 1 2 3 4
//
//  }
//}

//void callback_MotorPWM (const geometry_msgs::Quaternion  &motor) {
//  String ModeRobot = parseModeRobot(MODE_ROBOT_NOW.toInt());
//  if (ModeRobot == DEBUG_ROS_PWM) {
//    settingPWM[0] = motor.x;
//    settingPWM[1] = motor.y;
//    settingPWM[2] = motor.z;
//    settingPWM[3] = motor.w;
//  }
//}
//void callback_ModeDebug (const std_msgs::String &mode_robot) {
//  MODE_ROBOT_NOW = mode_robot.data;
//}
//

//
//void callback_Motor2 (const robot_riset::MotorEncoder &motor) {
//  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
//    debug_PWM_Motor[1] = motor.pwm;
//    debug_Arah_Motor[1] = motor.direction;
//  }
//   if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
//    debug_RPM_Motor[1] = motor.rpm;
//    debug_Arah_Motor[1] = motor.direction;
//  }
//}
//
//void callback_Motor3 (const robot_riset::MotorEncoder &motor) {
//  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
//    debug_PWM_Motor[2] = motor.pwm;
//    debug_Arah_Motor[2] = motor.direction;
//  }
//   if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
//    debug_RPM_Motor[2] = motor.rpm;
//    debug_Arah_Motor[2] = motor.direction;
//  }
//}
//void callback_Motor4 (const robot_riset::MotorEncoder &motor) {
//  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
//    debug_PWM_Motor[3] = motor.pwm;
//    debug_Arah_Motor[3] = motor.direction;
//  }
//   if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
//    debug_RPM_Motor[3] = motor.rpm;
//    debug_Arah_Motor[3] = motor.direction;
//  }
//}
//
//void print_Mode(String mode) {
//  if (mode == DEBUG_ROS_PWM) {
//    if (msTimer.fire()) Robot.loginfo("MODE : Debug Menggunakan ROS");
//  }
//}
//
//char * ROS_Int2Char(int angka) {
//  char cstr[16];
//  itoa(angka, cstr, 10);
//
//  return cstr;
//}

//void loadAllParam() {
//  if (!Robot.getParam("/robot_riset/param/pid/m1/kp", &m1_kp)) {
//    strcpy(msgbuf, "Failed to read /robot_riset/param/pid/m1/kp");
//  }
//  if (!Robot.setParam("/robot_riset/param/pid/m1/ki", &m1_kp)) {
//    strcpy(msgbuf, "Failed to read /robot_riset/param/pid/m1/kp");
//  }
//  Robot.loginfo(msgbuf);
//}
