// serial_configuration.ino
void enviarDatos(float current_left, float current_right, float voltage_12v, float voltage_5v) {
  Serial.print("DATA,");
  Serial.print(current_left, 2); Serial.print(",");
  Serial.print(current_right, 2); Serial.print(",");
  Serial.print(voltage_12v, 2); Serial.print(",");
  Serial.print(voltage_5v, 2);
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
      
      if (firstComma != -1 && secondComma != -1) {
        String rele_str = input.substring(firstComma + 1, secondComma);
        String update_rele_str = input.substring(secondComma + 1);
        
        uint8_t rele_cmd = (uint8_t)rele_str.toInt();
        bool update_rele = (update_rele_str.toInt() == 1);

        // Solo aplicar comandos de relés si update_rele es true
        if (update_rele) {
          // Aplicar el comando directamente a los relés (igual que en Rele_callback)
          digitalWrite(GPIO_RELE_M_LEFT,   rele_cmd & (1 << 0) ? HIGH : LOW);  
          digitalWrite(GPIO_RELE_M_RIGHT,  rele_cmd & (1 << 1) ? HIGH : LOW);  
          digitalWrite(GPIO_RELE_STOP,     rele_cmd & (1 << 2) ? HIGH : LOW);  
          digitalWrite(GPIO_RELE_LEFT,     rele_cmd & (1 << 3) ? HIGH : LOW);  
          digitalWrite(GPIO_RELE_RIGHT,    rele_cmd & (1 << 4) ? HIGH : LOW);  
          digitalWrite(GPIO_RELE_SAFETY,   rele_cmd & (1 << 5) ? HIGH : LOW);
          
          // Opcional: enviar ACK por serial
          // Serial.println("RELE_CMD_OK");
        }
      }
    }
  }
}
