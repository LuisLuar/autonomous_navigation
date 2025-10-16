
# Instrucciones para agregar bibliotecas personalizadas al Arduino IDE

## 🧩 Descripción
Este documento explica cómo copiar e instalar las bibliotecas personalizadas necesarias para tus proyectos en **Arduino IDE**.

---

## 📋 Pasos

1. **Abrir el Arduino IDE**

   Inicia el entorno de desarrollo de Arduino.

2. **Ir a las preferencias**

   En la barra de menú selecciona:

        File → Preferences

3. **Ubicar la carpeta del Sketchbook**

Dentro de la ventana de preferencias, busca el campo **Sketchbook location**.  
Por defecto, la ruta suele ser:

        /home/$USER/snap/arduino/current/Arduino


4. **Abrir la carpeta de bibliotecas**

Dentro de la ubicación anterior, accede a la carpeta:

        libraries


5. **Copiar las bibliotecas personalizadas**

Copia las siguientes carpetas dentro del directorio `libraries`:

        GP2Y0E03_ESP32
        imu_publisher
        motorControl
        odometry
        SabertoothSimplified


---

## ✅ Verificación

1. Reinicia el **Arduino IDE**.  
2. Abre el menú:

        Sketch → Include Library → Manage Libraries...

3. Verifica que las bibliotecas aparezcan en la lista o que puedas incluirlas en tu código con:
```cpp
#include <imu_publisher.h>
```


---

📁 Autor: Luis Raynel Vasquez Quishpe

📅 Última actualización: Octubre 2025