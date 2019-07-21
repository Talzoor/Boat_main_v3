
void US() {
  if (currentMillis - pMsUS >= intervalUS) { //Ultrasonic delay
    pMsUS = currentMillis;

    US_distance = echo_duration / 58;
    if (US_distance > 0.00 and US_distance <= 300.0) {    // fillter the obvious
      US_distance = US_DEMA_filter(US_distance);
    } else {    // Shut readings
      US_distance = -1;
    }
    Fill_arr(US_distance);
  } //end (currentMillis - pMsUS >= intervalUS)
  US_distance_cm = US_dist();
}

void US_dist_arr_INIT() {
  for (int i = 0; i < 10; i++) {
    US_distance_arr[i] = -1;
  }
}

void Fill_arr(float _data) {
  int i_FD;
  for (i_FD = (US_arr_size - 1); i_FD >= 0; i_FD--) {

    //    if (data_in[i_FD] != DATA_INIT) {
    US_distance_arr[i_FD + 1] = US_distance_arr[i_FD];
    //    }
  }
  if (i_FD == -1) {
    US_distance_arr[0] = _data;
  }

}

void print_US_arr() {
  int i_FD;
  for (i_FD = (US_arr_size - 1); i_FD >= 0; i_FD--) {
//    Serial.print("["); Serial.print(i_FD); Serial.print("]="); Serial.print(US_distance_arr[i_FD]); Serial.print(",");
  }
//  Serial.println();
}

bool US_arr_data_check(float _threshold, int _certainty) {
  int temp_counter = 0;
  bool return_val = false;
  for (int i_FD = (US_arr_size - 1); i_FD >= 0; i_FD--) {
    float US_read = US_distance_arr[i_FD];
    if (US_read != -1 and US_read <= _threshold) { // real distance and below threshold
      temp_counter++;
    }
  }
  if (temp_counter >= _certainty) return_val = true;
  return return_val;
}

int US_dist() {
  int certainty = 5;
  bool flag = false;
  int _dist;
  for (_dist = 30; _dist < 250 and !flag; _dist += 10) {
    flag = US_arr_data_check(_dist, certainty);
  }
  return _dist-10;
}

void timerIsr()
{
  trigger_pulse();                                 // Schedule the trigger pulses
  //  distance_flasher();                              // Flash the onboard LED distance indicator
}

void trigger_pulse()
{
  static volatile int state = 0;                 // State machine variable

  if (!(--trigger_time_count))                   // Count to 200mS
  { // Time out - Initiate trigger pulse
    trigger_time_count = TICK_COUNTS;           // Reload
    state = 1;                                  // Changing to state 1 initiates a pulse
  }

  switch (state)                                 // State machine handles delivery of trigger pulse
  {
    case 0:                                      // Normal state does nothing
      break;

    case 1:                                      // Initiate pulse
      digitalWrite(trigPin, HIGH);              // Set the trigger output high
      state = 2;                                // and set state to 2
      break;

    case 2:                                      // Complete the pulse
    default:
      digitalWrite(trigPin, LOW);               // Set the trigger output low
      state = 0;                                // and return state to normal 0
      break;
  }
}

void echo_interrupt()
{
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}
