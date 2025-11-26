// serial_configuration.ino
bool reset_cmd = false;

void enviarDatos() {
  Serial.print("DATA,");
  Serial.print(vx, 2); Serial.print(",");
  Serial.print(wz, 2); Serial.print(",");
  Serial.print(x_pos, 2); Serial.print(",");
  Serial.print(y_pos, 2); Serial.print(",");
  Serial.print(yaw_enc, 2); Serial.print(",");
  Serial.print(ax, 2); Serial.print(",");
  Serial.print(ay, 2); Serial.print(",");
  Serial.print(az, 2); Serial.print(",");
  Serial.print(gx, 2); Serial.print(",");
  Serial.print(gy, 2); Serial.print(",");
  Serial.print(gz, 2); Serial.print(",");
  Serial.print(roll_imu, 2); Serial.print(",");
  Serial.print(pitch_imu, 2); Serial.print(",");
  Serial.print(yaw_imu, 2); Serial.print(",");
  Serial.print(range_front, 2); Serial.print(",");
  Serial.print(range_left, 2); Serial.print(",");
  Serial.print(range_right, 2);
  Serial.println();
  Serial.flush();
}

void recibirDatos() {
  if (Serial.available() > 0) {
    last_serial_activity = millis();
    serial_connected = true;
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("CMD,")) {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);
      int fourthComma = input.indexOf(',', thirdComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1) {
        String linear_str = input.substring(firstComma + 1, secondComma);
        String angular_str = input.substring(secondComma + 1, thirdComma);
        String reset_str = input.substring(thirdComma + 1, fourthComma);
        String update_velocity_str = input.substring(fourthComma + 1);
        
        float linear = linear_str.toFloat();
        float angular = angular_str.toFloat();
        reset_cmd = (reset_str.toInt() == 1);
        bool update_velocity = (update_velocity_str.toInt() == 1);

        // Solo aplicar velocidades si update_velocity es true
        if (update_velocity) {
          float vI = linear - (angular * L) / 2;
          float vD = linear + (angular * L) / 2;
          Left.wRef = (2 * vI) / D;
          Right.wRef = (2 * vD) / D;
        }

        if (reset_cmd) {
          roll_imu = 0; pitch_imu = 0; yaw_imu = 0;
          x_pos = 0.0; y_pos = 0.0; yaw_enc= 0.0;
          reset_cmd = false;
          // puedes enviar ACK serial si quieres: Serial.println("RESET_OK");
        }
      }
    }
  }
}
