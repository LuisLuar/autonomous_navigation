
void error_loop() {
  unsigned long long last_error_time = millis();
    
  while (1) {

    ST.motor(1, 0);
    ST.motor(2, 0);
    
    digitalWrite(led_error, !digitalRead(led_error));
    //Serial.println("error..");
    delay(500);
    

    if (millis() - last_error_time > 3000) ESP.restart();
  }
}
void enviarDatos() {
  // Enviar todos los datos en un solo mensaje separados por comas
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
  Serial.println(); // Fin de línea
  
  // Añadir flush para asegurar envío
  Serial.flush();
}

void recibirDatos() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("CMD,")) {
      // Formato: CMD,linear,angular,reset
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        String linear_str = input.substring(firstComma + 1, secondComma);
        String angular_str = input.substring(secondComma + 1, thirdComma);
        String reset_str = input.substring(thirdComma + 1);
        
        // Conversión segura a float
        float linear = linear_str.toFloat();
        float angular = angular_str.toFloat();
        reset = (reset_str.toInt() == 1);

        //linear and angular velocities are converted to leftwheel and rightwheel velocities
        float vI = linear - (angular * L) / 2;
        float vD = linear + (angular * L) / 2;
      
        Left.wRef = (2 * vI) / D;
        Right.wRef = (2 * vD) / D;
        
        if (reset) {
          // Resetear posición si se solicita
          roll_imu = 0, pitch_imu = 0, yaw_imu = 0; //datos de la imu
          x_pos = 0.0, y_pos = 0.0, yaw_enc= 0.0; //datos del encoder
          reset = false; // Resetear la bandera
        }
        
        // Debug opcional
        // Serial.print("Received - Linear: ");
        // Serial.print(linear);
        // Serial.print(", Angular: ");
        // Serial.println(angular);
      }
    }
  }
}
