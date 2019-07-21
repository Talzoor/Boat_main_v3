

void Servo_Initialize() {
  PWM_module.begin();

  PWM_module.setPWMFreq(60);

  PWM_module.setPWM(SERVO_UP_DOWN, 0, servo_up_pos);
  PWM_module.setPWM(SERVO_LEFT_RIGHT, 0, servo_left_pos);

}

int Servo_speed(double _step, double _low_val , double _high_val) {
  double _tmp_val;

  double dist_from_zero = _low_val;
  _low_val = 0; _high_val = _high_val - dist_from_zero;
  _step = _step - dist_from_zero;
  double mid = _high_val / 2;

  if (_step <= mid) { // 0 --> 50 --> 100
    _tmp_val = mid * pow(_step / mid, 2.2);
  } else {
    _tmp_val = _high_val - mid * pow(abs(_step - _high_val) / mid, 2.2);
  }
  return dist_from_zero + _tmp_val;
}

//

void Servo_move() {
  if (servo_step == 0) servo_step = 1; //starting

  if (servo_step == 1) {
    if (servo_counter <= 10) {             // waiting step
      servo_counter++;
    } else {
      servo_step = 2;
      servo_counter = SERVO_UP_DOWN_MIN;
    }
  }
  else if (servo_step == 2) {
    if (servo_counter <= 250) {             // up
      int servo_move = Servo_speed(servo_counter, SERVO_UP_DOWN_MIN, 250);
      //        Serial.print("servo_move:"); Serial.print(servo_move);
      PWM_module.setPWM(SERVO_UP_DOWN, 0, servo_move);
      servo_counter++;
    } else {
      servo_step++;
      servo_counter = SERVO_LR_CENTER;
    }
  }
  else if (servo_step == 3) {             // right
    if (servo_counter >= 160) {
      int servo_move = Servo_speed(servo_counter, 160, SERVO_LR_CENTER);      // SERVO_LR_CENTER=345
      PWM_module.setPWM(SERVO_LEFT_RIGHT, 0, servo_move);
      servo_counter--;
    } else {
      servo_step++;   // =4 just when finishing circle
    }
  }
  else if (servo_step == 4) {             // left
    if (servo_counter <= 500) {
      int servo_move = Servo_speed(servo_counter, 160, 500);
      PWM_module.setPWM(SERVO_LEFT_RIGHT, 0, servo_move);
      servo_counter++;
    } else {
      servo_step++;
    }
  }
  else if (servo_step == 5) {             // back to center
    if (servo_counter >= SERVO_LR_CENTER) {
      int servo_move = Servo_speed(servo_counter, SERVO_LR_CENTER, 500);
      PWM_module.setPWM(SERVO_LEFT_RIGHT, 0, servo_move);
      servo_counter--;
    } else {
      servo_step++;
      servo_counter = 250;
    }
  } else if (servo_step == 6) {             // down
    if (servo_counter >= SERVO_UP_DOWN_MIN) {
      int servo_move = Servo_speed(servo_counter, SERVO_UP_DOWN_MIN, 250);
      PWM_module.setPWM(SERVO_UP_DOWN, 0, servo_move);
      servo_counter--;
    } else {
      servo_step++;
    }
  }
  else if (servo_step == 7) {
    servo_step = -1;
  }

}
