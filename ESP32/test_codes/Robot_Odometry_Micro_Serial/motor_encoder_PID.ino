//motor_encoder.ino
unsigned int ppr = 2000;         // Número de muescas que tiene el disco del encoder.

void beginEncoder(int LA,int LB,int LZ,int RA,int RB,int RZ){
  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Opcional
  
  encoderR.attachFullQuad(RA, RB);  // ✅ Configura el encoder global derecho
  encoderR.setCount(0);             // ✅ Inicializa el contador
  
  encoderL.attachFullQuad(LA, LB);  // ✅ Configura el encoder global izquierdo  
  encoderL.setCount(0);             // ✅ Inicializa el contador

  pinMode(LZ, INPUT_PULLUP);

  pinMode(RZ, INPUT_PULLUP);
  //Serial.println("Encoder Omron E6B2-CWZ3E listo.");
}

void beginPid(){
  motorL.setGains (0.7, 0.11, 0.03); // (Kc,Ti,Td)
  motorR.setGains (0.4, 0.11, 0.03); // (Kc,Ti,Td);

  motorL.setCvLimits(63, 0);
  motorR.setCvLimits(63, 0);

  motorL.setPvLimits(16.9, 0); //Leftuierdo 16.4 rad/s
  motorR.setPvLimits(16.9, 0); //derecho
  
  //Serial.println("Controlador PID configurado y listo.");
}

void encoderPID(){
  updateMotor(Left, encoderL, motorL, 1);
  updateMotor(Right, encoderR, motorR, 2);

  vx = (Right.v + Left.v) / 2;
  wz = (Right.v - Left.v) / L ; // rad/s

  // Filtro sencillo deadzone
  if (abs(vx) < 0.05) vx = 0;
  if (abs(wz) < 0.05) wz = 0;

  yaw_enc += wz * dt_enc / 1000; //radians

  float cos_h = cos(yaw_enc);
  float sin_h = sin(yaw_enc);
  float delta_x = vx * cos_h * dt_enc / 1000; //m
  float delta_y = vx * sin_h * dt_enc / 1000; //m

  x_pos += delta_x;//*0.85;
  y_pos += delta_y;//*1.1;
}

void updateMotor(DatosMotores &m, ESP32Encoder &enc, motorControl &ctrl, int id) {
  const float alpha = 0.8;
  long count = enc.getCount();
  enc.setCount(0);
  
  m.wSinFilter = count * (2 * PI * 1000) / (dt_enc * ppr);
  m.w = alpha * m.wSinFilter + (1-alpha) * m.wAnterior; // velocidad angular actual en rad/s
  m.wAnterior = m.w;
  
  m.v = (m.w * D) / 2;
  m.outValue = ctrl.compute(m.wRef, m.w);
  ST.motor(id, m.outValue);
}
