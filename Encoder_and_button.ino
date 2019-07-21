
void encoder_func() {
  encoder.tick();

  int newPos = encoder.getPosition();
  if (pos != newPos) {

    pos = newPos;

    Led_strip_off();
    encoder_mode = abs(pos) % 4;

  } // if (pos != newPos)
  button_func();
}

void button_func() {
  if (button.getSingleDebouncedPress())   //getSingleDebouncedRelease())
  {
    HEART_BEAT();

    bool PID_Debug = true;      // check for pid constant
    if (PID_Debug) {
      if (encoder_mode == 0) {
        prog_with_servo = false;
        if (FULL_PROG_STEP == -1) FULL_PROG_STEP = 0;
        Serial.print("FULL_PROG_STEP:"); Serial.println(FULL_PROG_STEP);

        if (mode == MODE_NOTHING) {

        }
        else {
          mode = MODE_NOTHING;
          //          kp++;
        }
      }
      if (encoder_mode == 1) {
        if (mode != MODE_SERVO_UD_SET) {
          PWM_module.setPWM(SERVO_UP_DOWN, 0, SERVO_UP_DOWN_MIN + 10);
          mode = MODE_SERVO_UD_SET;
        }
        else {
          PWM_module.setPWM(SERVO_UP_DOWN, 0, SERVO_UP_DOWN_MIN);
          mode = 0;
        }

      }
      if (encoder_mode == 2) {
        prog_with_servo = true;
        if (FULL_PROG_STEP == -1) FULL_PROG_STEP = 0;
      }
      if (encoder_mode == 3) {
        mode = MODE_NOTHING;
        if (servo_step == 3) servo_step = 4;
      }

    }

  } //  if (button.getSingleDebouncedPress())
}


void run_PID_calbiration() {

}
