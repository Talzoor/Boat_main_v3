

void Blue_Serial_Rec() {

  if (BlueSerial.available() > 0 )
  {
    char c;
    while ((BlueSerial.available() > 0) && (g_command_ready == false)) {
      c = (char)BlueSerial.read();
      if (c == ';') {
        g_command_ready = true;
      }
      else {
        g_command += c;
      }

    }

    if (g_command_ready) {
      char tmp_str[200] = "";
      g_command.toCharArray(tmp_str, 40);
      strcat(tmp_str, ";");

      BlueSerial.write(tmp_str); // ECHO
      Serial.print(F("ECHO:"));
      Serial.println(tmp_str);

      Serial.print(g_command);  //end of line and function exit

      if (g_command == "L0") {
        digitalWrite(13, LOW);
      }
      else if (g_command == "L1") {
        digitalWrite(13, HIGH);
      }
      else if (g_command == "FILM") {
        if (servo_step == -1) servo_step = 0;
        servo_counter = 0;
      }
      else if (g_command == "mode_PID") {
        send_angle = true;
        mode = MODE_PID;
      }
      else if (g_command == "mode_test") {
        mode = MODE_TEST;
        motors_off();
      }
      else if (g_command == "mode_wheel") {
        motor1_speed_out = 0;
        motor2_speed_out = 0;
        mode = MODE_WHEEL;
        motors_runing = true;
      }
      else if (g_command == "angle_stop") {
        send_angle = false;
      }
      else if (g_command == "motors_off") {
        motors_runing = false;
      }

      else if (g_command == "motors_on") {
        motors_runing = true;
      }

      else if (g_command == "set_angle") {
        Setpoint = Input;
        send_angle = true;
        mode = MODE_PID;
      }


      // A one-parameter command...
      else if (g_command.startsWith("servo1")) {
        int pos = g_command.indexOf(" ");
        if (pos == -1) {
          Serial.println(F("Error: Command 'read_page' expects an int operand"));
        } else {
          int page = (word)g_command.substring(pos).toInt();
          Serial.print(F("word="));
          Serial.print(page);
          Serial.println(F(";"));
          uint16_t pulselen = map (page, 0 , 180, 0, 4095);
          Serial.print(F("pulselen:")); Serial.println(pulselen);
          PWM_module.setPWM(SERVO_UP_DOWN, 0, pulselen);
          previousMillis = currentMillis; //for servo detach later

        }
      }

      else if (g_command.startsWith("servo2")) {
        int pos = g_command.indexOf(" ");
        if (pos == -1) {
          Serial.println(F("Error: Command 'read_page' expects an int operand"));
        } else {
          int page = (word)g_command.substring(pos).toInt();
          Serial.print(F("word="));
          Serial.print(page);
          Serial.println(F(";"));
          uint16_t pulselen = map (page, 0 , 180, 400, 600);
          PWM_module.setPWM(SERVO_LEFT_RIGHT, 0, pulselen);

          previousMillis = currentMillis; //for detach later
        }
      }

      else if (g_command.startsWith("motor1")) {
        int pos = g_command.indexOf(" ");
        if (pos == -1) {
          Serial.println(F("Error: Command 'read_page' expects an int operand"));
        } else {
          int page;
          byte idx = (word)g_command.indexOf('-');
          if (idx != 0 and idx != 255) {
            page = (-1) * (word)g_command.substring(idx + 1).toInt();
          } else {
            page = (word)g_command.substring(pos).toInt();
          }
          motor1_speed_out = page; //map(page,-255, 255, MIN_MOTOR_1, motorR_max_speed);
          motors_runing = true;
        }
      }

      else if (g_command.startsWith("motor2")) {
        int pos = g_command.indexOf(" ");
        if (pos == -1) {
          Serial.println(F("Error: Command 'read_page' expects an int operand"));
        } else {
          int page;
          byte idx = (word)g_command.indexOf('-');
          if (idx != 0 and idx != 255) {
            page = (-1) * (word)g_command.substring(idx + 1).toInt();
          } else {
            page = (word)g_command.substring(pos).toInt();
          }
          motor2_speed_out = page; //map(page,-255, 255, MIN_MOTOR_1, motorR_max_speed);
          motors_runing = true;
        }
      }

      else if (g_command.startsWith("MAX_SPEED")) {
        int pos = g_command.indexOf(" ");
        if (pos == -1) {
          Serial.println(F("Error(MAX_SPEED): Command 'read_page' expects an int operand"));
        } else {
          int page = (word)g_command.substring(pos).toInt();
          set_motors_max(page, page);
        }
      }

      else if (g_command.startsWith("P")) {
        String str_pid;
        int pos = g_command.indexOf(" ");
        if (pos == -1) {
          Serial.println(F("Error(PIDconst): Command 'read_page' expects an int operand"));
        } else {
          str_pid = g_command.substring(pos);
        }
        send_angle = true;
        mode = MODE_PID;

        String st1 = getValue(str_pid, ',', 0);
        String st2 = getValue(str_pid, ',', 1);
        String st3 = getValue(str_pid, ',', 2);

        if (isNumeric(st1) && isNumeric(st2) && isNumeric(st3)) {
          double Kp_val = st1.toDouble();
          double Kd_val = st2.toDouble();
          double Ki_val = st3.toDouble();

          kp = Kp_val / 10.0;
          kd = Kd_val / 10.0;
          ki = Ki_val / 10.0;
          PID_obj.setGains(kp, ki, kd);
          PID_obj.reset();
        }
      }

      else if (g_command.startsWith("wheel")) {
        int pos1 = g_command.indexOf("(");
        int pos2 = g_command.indexOf(",");
        int pos3 = g_command.indexOf(")");
        if (pos1 == -1) {
          Serial.println(F("Error(wheel): Command 'read_page' expects an int operand"));
        } else {
          wheel_x = (word)g_command.substring(pos1 + 1, pos2).toInt();
          wheel_y = (word)g_command.substring(pos2 + 1, pos3).toInt();
          mode = MODE_WHEEL;
        }
      }

done:
      g_command = "";
      g_command_ready = false;
      Serial.println();
    }
  }
}


void Blue_Serial_Send(byte mode, float data_1, float data_2, float data_3, float data_4) {
  char tmp_str[40] = "";
  char res[8]; // Buffer big enough for 7-character float
  if (mode == MODE_ONE_ANGLE) {
    strcat(tmp_str, "angle,");
    dtostrf(data_1, 6, 2, res); //w
    strcat(tmp_str, res);
    strcat(tmp_str, ",END");
  } else if (mode == MODE_ANGLE_SETPOINT) {
    strcat(tmp_str, "angle,");
    dtostrf(data_1, 6, 2, res); //w
    strcat(tmp_str, res);
    strcat(tmp_str, ",setpoint,");
    dtostrf(data_2, 6, 2, res); //w
    strcat(tmp_str, res);
    strcat(tmp_str, ",END");
  }
  else {
    if (!debug_sent_bt_data) {

#ifdef OUTPUT_READABLE_QUATERNION
      strcat(tmp_str, "QUAT,");
      dtostrf(data_1, 6, 2, res); //w
      strcat(tmp_str, res);
      strcat(tmp_str, ",");
      dtostrf(data_2, 6, 2, res); //x
      strcat(tmp_str, res);
      strcat(tmp_str, ",");
      dtostrf(data_3, 6, 2, res); //y
      strcat(tmp_str, res);
      strcat(tmp_str, ",");
      dtostrf(data_4, 6, 2, res); //z
      strcat(tmp_str, res);
      strcat(tmp_str, ",END");
      Serial.print(F("str:"));
      Serial.println(tmp_str);
#endif

#ifdef OUTPUT_READABLE_EULER
      strcat(tmp_str, "ANGLE,");
      dtostrf(data_1, 6, 2, res); //euler[0] * 180 / M_PI
      strcat(tmp_str, res);
      strcat(tmp_str, ",");
      dtostrf(data_2, 6, 2, res); //euler[1] * 180 / M_PI
      strcat(tmp_str, res);
      strcat(tmp_str, ",");
      dtostrf(data_3, 6, 2, res); //euler[2] * 180 / M_PI
      strcat(tmp_str, res);
      strcat(tmp_str, ",END");
      // print only relevant angle

#endif

    }

  }
  BlueSerial.write(tmp_str);
}
