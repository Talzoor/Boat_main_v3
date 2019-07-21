
void mpu_Serial() {
  if (Serial.available() > 0 )
  {
    char c;
    while (Serial.available() > 0 ) {
      c = (char)Serial.read();
      if (c == ';') {
        g_command_ready_mpu = true;
      }
      else {
        g_command_mpu += c;
      }
    }


    if (g_command_ready_mpu) {
      //      Serial.print(F("mpu_Serial:"));
      //      Serial.println(g_command_mpu);
      if (g_command_mpu.startsWith("a")) {    // angle
        int pos = g_command_mpu.indexOf("a") + 1;
        if (pos == -1) {
          Serial.println(F("Error: Command 'read_page' expects an int operand"));
        } else {
          double page = g_command_mpu.substring(pos).toDouble();
          angle_1 = page;
          angle_1 = Full_360_angle(angle_1);
          angle_1_filtered = DEMA_filter(angle_1);
        }
      }
      else if (g_command_mpu.startsWith("t")) { // Temerature
        int pos = g_command_mpu.indexOf("t") + 1;
        if (pos == -1) {
          Serial.println(F("Error: Command 'read_page' expects an int operand"));
        } else {
          double page = g_command_mpu.substring(pos).toDouble();
          mpu_temperature = page;
        }
      }

done:
      g_command_mpu = "";
      g_command_ready_mpu = false;
    }

  }
}
