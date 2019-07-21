
void PID_TUNE() {

  PIDAutotuner tuner = PIDAutotuner();
  long loopInterval = interval2 * 1000;
  // Set the target value to tune to
  // This will depend on what you are tuning. This should be set to a value within
  // the usual range of the setpoint. For low-inertia systems, values at the lower
  // end of this range usually give better results. For anything else, start with a
  // value at the middle of the range.
  tuner.setTargetInputValue(Setpoint);

  // Set the loop interval in microseconds
  // This must be the same as the interval the PID control loop will run at
  tuner.setLoopInterval(loopInterval);

  // Set the output range
  // These are the maximum and minimum possible output values of whatever you are
  // using to control the system (analogWrite is 0-255)
  tuner.setOutputRange(-255, 255);

  // Set the Ziegler-Nichols tuning mode
  // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
  // or PIDAutotuner::ZNModeNoOvershoot. Test with ZNModeBasicPID first, but if there
  // is too much overshoot you can try the others.
  tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);

  motors_runing = true;
  // This must be called immediately before the tuning loop
  tuner.startTuningLoop();

  // Run a loop until tuner.isFinished() returns true
  long microseconds;
  while (!tuner.isFinished()) {

    // This loop must run at the same speed as the PID control loop being tuned
    long prevMicroseconds = microseconds;
    microseconds = micros();

    // Get input value here (temperature, encoder position, velocity, etc)
    mpu_Serial();
    Input = angle_1_filtered;

    // Call tunePID() with the input value
    Output = tuner.tunePID(Input);

    // Set the output - tunePid() will return values within the range configured
    // by setOutputRange(). Don't change the value or the tuning results will be
    // incorrect.
    motors_wheel(Output, 255);
    motors();

//    Serial_Print_data(motor1_speed_out, motor2_speed_out, 0); //distance);
    // This loop must run at the same speed as the PID control loop being tuned
    while (micros() - microseconds < loopInterval) delayMicroseconds(1);
  }

  // Turn the output off here.
  motors_wheel(0, 255);
  motors_runing = false;
  motors_off();


  // Get PID gains - set your PID controller's gains to these
  double kp = tuner.getKp();
  double ki = tuner.getKi();
  double kd = tuner.getKd();
  Serial.print(F("kp:")); Serial.print(kp);
  Serial.print(F(" ki:")); Serial.print(ki);
  Serial.print(F(" kd:")); Serial.println(kd);
  EEPROM_Write_double(kp, ki, kd);

  motors_off();
//  while (1);

}
