void DO_FULL_PROGRAM() {
  switch (FULL_PROG_STEP) {
    case 0:
      pMsStartALL = currentMillis;
      mode = MODE_PID;
      Setpoint = Input;
      PID_obj.reset();
      PID_obj.setGains(kp, ki, kd);
      motors_runing = true;
      time_started = millis();
      FULL_PROG_STEP++;
      break;

    case 1:

      sea_distance_calc = V_scalar * (currentMillis - time_started) / 1000;
      if (sea_distance_calc > (15 * 1000)) { // >15m
        if (!speed_reduced) {
          motorL_max_speed = motorL_max_speed * 0.8;
          motorR_max_speed = motorR_max_speed * 0.8;
          speed_reduced = true;
        }
      }
      if (US_distance_cm <= 200) {
        FULL_PROG_STEP++;
      }
      break;

    case 2:
      if (prog_with_servo) servo_step = 0;
      if (Setpoint > 90) far_circle_angle = Setpoint - 90;
      else far_circle_angle = Setpoint + 270;
      if (Setpoint > 180) Setpoint = Setpoint - 180;
      else Setpoint = Setpoint + 180;
      boat_speed = 170;
      boat_turn = 50;
      mode = MODE_TURN1;
      FULL_PROG_STEP++;
      break;

    case 3:
      if (turn_val < 20) {
        FULL_PROG_STEP++;
      }
      break;

    case 4:
      mode = MODE_TURN2;
      FULL_PROG_STEP++;
      break;

    case 5:
      //      if (Input < (far_circle_angle + 30) and Input > (far_circle_angle - 30)) {
      //        if (speed_reduced)  {
      ////          mode = MODE_GO_STRAIT;
      //////          pMsStartSTRAIGHT = currentMillis;
      ////          motorR_max_speed = MAX_HIGH * 0.87;
      ////          motorL_max_speed = MAX_HIGH * 1;
      //          speed_reduced = false;
      //        }
      //      } else if (!speed_reduced) {
      //        motorL_max_speed = motorL_max_speed * 0.8;
      //        motorR_max_speed = motorR_max_speed * 0.8;
      //        speed_reduced = true;
      //      }
      if (Input < (Setpoint + 20) and Input > (Setpoint - 20)) {         // Setpoint = 50 --> 40-60;  Input = 60-->50-->40        = 40-->50-->60
        FULL_PROG_STEP++;
      }
      break;

    case 6:
      if (prog_with_servo) servo_step = 4;
      mode = MODE_PID;
      if (speed_reduced) {
        motorR_max_speed = MAX_HIGH * 0.87;
        motorL_max_speed = MAX_HIGH * 1;
        speed_reduced = false;
      }
      PID_obj.reset();
      FULL_PROG_STEP++;
      break;

    case 7:
      if (US_distance_cm <= 50) {
        FULL_PROG_STEP++;
      }
      break;

    case 8:     // end
      FULL_PROG_STEP = -1;
      mode = MODE_NOTHING;
      break;
  }
}



void DO_FULL_PROGRAM2() {
  switch (FULL_PROG_STEP) {
    case 0:
      mode = MODE_PID;
      Setpoint = Input;
      PID_obj.reset();
      PID_obj.setGains(kp, ki, kd);
      motors_runing = true;
      FULL_PROG_STEP++;
      break;

    case 1:
      if (US_distance_cm <= 40) {
        FULL_PROG_STEP++;
      }
      break;

    case 2:
      if (prog_with_servo) servo_step = 0;
      mode = MODE_TURN2;
      FULL_PROG_STEP++;
      break;

    case 3:
      if (servo_step == -1) {
        FULL_PROG_STEP++;
        mode = MODE_PID;
      }
      break;
    case 4:
      if (US_distance_cm <= 40) {
        FULL_PROG_STEP++;
      }
      break;
    case 5:
      FULL_PROG_STEP = -1;
      motors_off();
      mode = MODE_NOTHING;
      break;

  }
}
