void Init_IMU() {
   if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      delay(3000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

  imu.setGyroFSR(2000);
  imu.setAccelFSR(2);
  imu.setLPF(5); // Set LPF corner frequency to 5Hz // Can be any of the following: 188, 98, 42, 20, 10, 5
  imu.setSampleRate(10); // Set sample rate to 10Hz


  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10);
}

void Read_IMU() {
  if ( imu.dataReady() ) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO);
    gX = imu.calcGyro(imu.gx);
    gY = imu.calcGyro(imu.gy);
    gZ = imu.calcGyro(imu.gz);

    aX = imu.calcAccel(imu.ax);
    aY = imu.calcAccel(imu.ay);
    aZ = imu.calcAccel(imu.az);

  }


  if ( imu.fifoAvailable() )  {
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      imu.computeEulerAngles();

      q0 = imu.calcQuat(imu.qw);
      q1 = imu.calcQuat(imu.qx);
      q2 = imu.calcQuat(imu.qy);
      q3 = imu.calcQuat(imu.qz);

      Yaw = imu.yaw;
      Pitch = imu.pitch;
      Roll = imu.roll;

      IMU_Time = imu.time;

    }
  }


}
