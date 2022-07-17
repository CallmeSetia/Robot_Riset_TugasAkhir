
// -- FUNCTION BIASA
String parseString(String data, char separator[], int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator[0] || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


int getModeRobot() {
  if ( MODE_ROBOT_NOW == ROBOT_RUN) return 1;
  if ( MODE_ROBOT_NOW == ROBOT_STOP) return 2;
  if ( MODE_ROBOT_NOW == DEBUG_ROBOT_RPM) return 3;
  if ( MODE_ROBOT_NOW == DEBUG_ROBOT_PWM) return 4;
}

String parseModeRobot(int mode) {
  String res = "";
  switch (mode) {
    case 1 :
      res = String("robot_run");
      break;
    case 2 :
      res = String("robot_stop");
      break;
    case 3 :
      res = String("debug_rpm");
      break;
    case 4:
      res = String("debug_pwm");
      break;
      
    default:
      break;
  }
  return res;
  //  if ( MODE_ROBOT_NOW == mode) return String("robot_run");
  //  if ( MODE_ROBOT_NOW == mode) return  String("robot_stop");
  //  if ( MODE_ROBOT_NOW == mode) return String("debug_rpm");
  //  if ( MODE_ROBOT_NOW == mode) return String("debug_pwm");
}
