// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

int Range_to_Power_percentage(int range) { // return 0 -> 100 %
  int tmp_power = 100;
  if (range < 100) {
    if (range > 80) {
      tmp_power = 50;
    } else {    // 0>range>80
      tmp_power = 0;
    }
  }
  return tmp_power;
}

void Serial_print_matlab(double angle_1, double angle_1_filtered) {
  char tmp_str[40];
  char res[8];
  strcpy(tmp_str, "");
  strcat(tmp_str, "A(");
  dtostrf(angle_1, 6, 2, res);
  strcat(tmp_str, res);
  strcat(tmp_str, ") ");
  //      Serial.print(tmp_str);
  //      Serial.println();

  //      strcpy(tmp_str, "");
  strcat(tmp_str, "B(");
  dtostrf(angle_1_filtered, 6, 2, res);
  strcat(tmp_str, res);
  strcat(tmp_str, ")");
  Serial.print(tmp_str);
  Serial.println();
}

float Generate_noise() {
  long randNumber = random(30);               //noise
  return randNumber;
}

void SetInOut_pins() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  //  pinMode(SERVO1_PIN, OUTPUT);
  //  pinMode(10, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  //  pinMode(SERVO1_PIN, OUTPUT);
  //  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(trigPin, OUTPUT); //set pinmodes
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoPin), echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
}

void HEART_BEAT() { //every sec
  if (HB) {
    HB = false;
  } else {
    HB = true;
  }
  digitalWrite(ONBOARD_LED_PIN, HB);
}

boolean isNumeric(String str) {
  unsigned int stringLength = str.length();

  if (stringLength == 0) {
    return false;
  }

  boolean seenDecimal = false;

  for (unsigned int i = 0; i < stringLength; ++i) {
    if (isDigit(str.charAt(i))) {
      continue;
    }
    if (str.charAt(i) == ' ') {
      continue;
    }
    if (str.charAt(i) == '.') {
      if (seenDecimal) {
        return false;
      }
      seenDecimal = true;
      continue;
    }
    return false;
  }
  return true;
}

int rgb_led_pwm_power(unsigned char _val) {
  int temp_val = map(_val, 0, 100, 4095, 0);
  return temp_val;
}

int led_pwm_power(unsigned char _val) {
  int temp_val = map(_val, 0, 100, 0, 4095);
  return temp_val;
}

void PWM_Led_off() {
  PWM_module.setPWM(LED_BLUE, 0, rgb_led_pwm_power(0));
  PWM_module.setPWM(LED_GREEN, 0, rgb_led_pwm_power(0));
  PWM_module.setPWM(LED_RED, 0, rgb_led_pwm_power(0));
}

void Led_strip_off() {
  PWM_module.setPWM(LED_STRIP_RIGHT, 0, led_pwm_power(0));
  PWM_module.setPWM(LED_STRIP_CENTER, 0, led_pwm_power(0));
  PWM_module.setPWM(LED_STRIP_LEFT, 0, led_pwm_power(0));
}

void Led_strip_indication(unsigned char _led, unsigned char _power) {
  unsigned char _pin = _led + 4;    // 0, 1, 2 --> 4, 5, 6
  //  Led_strip_off();
  if (_pin < 7) {
    PWM_module.setPWM(_pin, 0, led_pwm_power(_power));    // blue wire
  }
  else {
    led_state = !led_state;
    PWM_module.setPWM(4, 0, led_pwm_power(_power));
    PWM_module.setPWM(5, 0, led_pwm_power(_power));
    PWM_module.setPWM(6, 0, led_pwm_power(_power));
  }
}

void Led_blink(int _pin, int _times) {
  for (int i = 0; i < _times; i++) {
    PWM_module.setPWM(_pin, 0, led_pwm_power(50));
    delay(250);
    PWM_module.setPWM(_pin, 0, led_pwm_power(0));
    delay(250);
  }
}

double Full_360_angle(double _angle) {
  if (_angle < 0) { // for negative angle
    _angle = 180 + map(_angle, -180, 0, 0, 180);
  }
  return _angle;
}
