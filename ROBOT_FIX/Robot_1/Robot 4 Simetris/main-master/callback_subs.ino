void callback_ModeDebug (const std_msgs::String &mode_robot) {
  MODE_ROBOT_NOW = mode_robot.data;
}

void callback_Motor1 (const robot_riset::MotorEncoder &motor) {
  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
    debug_PWM_Motor[0] = motor.pwm;
    debug_Arah_Motor[0] = motor.direction;
  }
  if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
    debug_RPM_Motor[0] = motor.rpm;
    debug_Arah_Motor[0] = motor.direction;
  }
}

void callback_Motor2 (const robot_riset::MotorEncoder &motor) {
  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
    debug_PWM_Motor[1] = motor.pwm;
    debug_Arah_Motor[1] = motor.direction;
  }
   if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
    debug_RPM_Motor[1] = motor.rpm;
    debug_Arah_Motor[1] = motor.direction;
  }
}

void callback_Motor3 (const robot_riset::MotorEncoder &motor) {
  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
    debug_PWM_Motor[2] = motor.pwm;
    debug_Arah_Motor[2] = motor.direction;
  }
   if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
    debug_RPM_Motor[2] = motor.rpm;
    debug_Arah_Motor[2] = motor.direction;
  }
}
void callback_Motor4 (const robot_riset::MotorEncoder &motor) {
  if (MODE_ROBOT_NOW == DEBUG_ROS_PWM) {
    debug_PWM_Motor[3] = motor.pwm;
    debug_Arah_Motor[3] = motor.direction;
  }
   if (MODE_ROBOT_NOW == DEBUG_ROS_RPM) {
    debug_RPM_Motor[3] = motor.rpm;
    debug_Arah_Motor[3] = motor.direction;
  }
}

void print_Mode(String mode) {
  if (mode == DEBUG_ROS_PWM) {
    if (msTimer.fire()) Robot.loginfo("MODE : Debug Menggunakan ROS");
  }
}

char * ROS_Int2Char(int angka) {
  char cstr[16];
  itoa(angka, cstr, 10);

  return cstr;
}
