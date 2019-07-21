

void run_motor(byte pinA, byte pinB, byte pinPWM, int motor_speed) {
  if (motor_speed < 0) { //reverse
    digitalWrite(pinA, HIGH) ;
    digitalWrite(pinB, LOW) ;
    //    analogWrite(pinPWM, 255 - abs(motor_speed));
  } else {              //forward
    digitalWrite(pinA, LOW) ;
    digitalWrite(pinB, HIGH) ;
  }
  analogWrite(pinPWM, abs(motor_speed));

}


void motors_wheel(int _left_right, int _forward_back) {
  int left_right;
  int forward_back;
  //check limits

  left_right = constrain(_left_right, -255, 255);
  forward_back = constrain(_forward_back, -255, 255);

  //take care for y (forward/back)

  //  motor1_speed_out = motorR_max_speed * (forward_back / 100);
  //  motor2_speed_out = motorL_max_speed * (forward_back / 100);

  //take care for x (left/right)
  if (left_right <= 0) { //turn right
    motor1_speed_out = ((255.0 - abs(left_right)) / 255.0) * forward_back;
    motor2_speed_out = forward_back;
  } else {              //turn left
    motor1_speed_out = forward_back;
    motor2_speed_out = ((255 - abs(left_right)) / 255.0) * forward_back;
  }
}

void motors_spin(int _left_right, int boat_speed) {
  int left_right;
  //check limits

  left_right = constrain(_left_right, -boat_speed, boat_speed);
  int abs_left_right = abs(left_right);

  //take care for x (left/right)
  if (left_right <= 0) { //turn right
    motor1_speed_out = abs_left_right;
    motor2_speed_out = -abs_left_right;
  } else {              //turn left
    motor1_speed_out = -abs_left_right;
    motor2_speed_out = abs_left_right;
  }
}

void motors_off() {
  motor1_speed_out = 0;
  motor2_speed_out = 0;
  run_motor(PIN_MotorR_A, PIN_MotorR_B, PIN_pwm_1, 0);
  run_motor(PIN_MotorL_A, PIN_MotorL_B, PIN_pwm_2, 0);
}

void set_motors_max(int new_max1, int new_max2) {
  motorR_max_speed = constrain(new_max1, MIN_MOTOR_1, MAX_HIGH);
  motorL_max_speed = constrain(new_max2, MIN_MOTOR_2, MAX_HIGH);
}

void map_and_constrain_motors() {
  if (motor1_speed_out >= 0) {
    constrainedOutput_motor1 = map(motor1_speed_out, 0, 255, MIN_MOTOR_1, motorR_max_speed);
  } else {
    constrainedOutput_motor1 = map(motor1_speed_out, -255, 0, (-1) * motorR_max_speed, (-1) * MIN_MOTOR_1);
  }
  if (motor2_speed_out >= 0) {
    constrainedOutput_motor2 = map(motor2_speed_out, 0, 255, MIN_MOTOR_2, motorL_max_speed);
  } else {
    constrainedOutput_motor2 = map(motor2_speed_out, -255, 0, (-1) * motorL_max_speed, (-1) * MIN_MOTOR_2);
  }
  if (motor1_speed_out >= 0) {
    constrainedOutput_motor1 = constrain(constrainedOutput_motor1, MIN_MOTOR_1, motorR_max_speed);
  } else {
    constrainedOutput_motor1 = constrain(constrainedOutput_motor1, (-1) * motorR_max_speed, (-1) * MIN_MOTOR_1);
  }
  if (motor2_speed_out >= 0) {
    constrainedOutput_motor2 = constrain(constrainedOutput_motor2, MIN_MOTOR_2, motorL_max_speed);
  } else {
    constrainedOutput_motor2 = constrain(constrainedOutput_motor2, (-1) * motorL_max_speed, (-1) * MIN_MOTOR_2);
  }
}

void motors() {
  if (motors_runing == true) {
    //      set_motors_max(motorR_max_speed, motorL_max_speed);
    map_and_constrain_motors();
    run_motor(PIN_MotorR_B, PIN_MotorR_A, PIN_pwm_1, constrainedOutput_motor1);  //run motor with constrained speed
    run_motor(PIN_MotorL_A, PIN_MotorL_B, PIN_pwm_2, constrainedOutput_motor2);
  } else {
    motors_off();
  }
}

int boat_local_turn(float _angle, double _Input) {    // positive angle is right angle, negative is left
  int tmp_val = 0;
  if (angle_to_turn != 999) {
    float delta = Full_360_angle(angle_to_turn - _Input);

    if (delta <= boat_turn + 5) {
      tmp_val = map(abs(delta), 0, boat_turn, 20, 255);
    } else {
      tmp_val = map(360 - abs(delta), 0, boat_turn, 20, -255);
    }

  }

  else { // first var value
    angle_to_turn = _Input + _angle;
    tmp_val = 255;

  }
  return tmp_val;
}
