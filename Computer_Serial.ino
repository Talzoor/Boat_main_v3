
void Serial_Print_data(int motor1_speed_out, int motor2_speed_out, int distance) {
  char tmp_str[40] = "";
  char res[8];
  strcat(tmp_str, "");
  dtostrf(Input, 6, 1, res);
  strcat(tmp_str, res);
  strcat(tmp_str, ",");
  dtostrf(Setpoint, 6, 1, res);
  strcat(tmp_str, res);
  strcat(tmp_str, ",");
  dtostrf(Output, 6, 1, res);
  strcat(tmp_str, res);

  Serial.print(F("In"));
  Serial.print(F(",Set"));
  Serial.print(F(",Out="));
  //    Serial.print(angle_1raw);
  //    Serial.print(",");
  Serial.print(tmp_str);
  //  Serial.print(F(","));
  //  Serial.print(Setpoint);
  //  Serial.print(F(","));
  //  Serial.print(Output);
  Serial.print(F(" m1="));
  Serial.print(motor1_speed_out);
  Serial.print(F(","));
  Serial.print(constrainedOutput_motor1);
  if (motors_runing) {
    Serial.print(F("!"));
  }
  Serial.print(F(" m2="));
  Serial.print(motor2_speed_out);
  Serial.print(F(","));
  Serial.print(constrainedOutput_motor2);
  if (motors_runing) {
    Serial.print(F("!"));
  }

  Serial.print(F(" LR:"));
  Serial.print(left_right_move);
  Serial.print(F(" FB:"));
  Serial.print(forward_back_move);
//  Serial.print(F(" dist="));
//  Serial.print(US_distance);
//  Serial.print(F("cm"));
Serial.print(F(" s.step="));
  Serial.print(servo_step);

  Serial.print(F(" US_dist_cm="));
  Serial.print(US_distance_cm);

  Serial.print(F(" mode:"));
  Serial.print(mode);
  Serial.print(F(" ecc_m:"));
  Serial.print(encoder_mode);

  Serial.print(F(" T_c:"));
  Serial.print(mpu_temperature);

  Serial.print(F(" F_STEP:"));
  Serial.print(FULL_PROG_STEP);

  Serial.print(F(" s_dis_c:"));
  Serial.print(sea_distance_calc);

  Serial.print(F(" E:"));
  Serial.print(currentMillis - pMsStartALL);




  Serial.println();
}
