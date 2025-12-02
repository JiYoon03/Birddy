// Birddy — sensor data acquisition (MAX30102 + GSR) and transmission via UART to main ESP32
// Libraries required:
//   - MAX30105 by SparkFun 
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// ---------------- Configuration ----------------
#define GSR_PIN 34
#define UART_TX 4
#define UART_RX 5

HardwareSerial SensorSerial(2);

long irValue = 0;
long redValue = 0;

void setup() {
  Serial.begin(115200);
  SensorSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  Wire.begin(21, 22);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
}

void loop() {

  if (particleSensor.check()) {
    irValue = particleSensor.getIR();
    redValue = particleSensor.getRed();
  }

  int gsrValue = analogRead(GSR_PIN);

  int hr = 0;
  int spo2 = 0;

  if (irValue > 20000) {
    hr = map(irValue, 20000, 100000, 60, 100);
  }

  if (redValue > 20000) {
    spo2 = map(redValue, 20000, 100000, 90, 100);
  }

  SensorSerial.print("{\"ir\":");
  SensorSerial.print(irValue);
  SensorSerial.print(",\"red\":");
  SensorSerial.print(redValue);
  SensorSerial.print(",\"hr\":");
  SensorSerial.print(hr);
  SensorSerial.print(",\"spo2\":");
  SensorSerial.print(spo2);
  SensorSerial.print(",\"gsr\":");
  SensorSerial.print(gsrValue);
  SensorSerial.println("}");

  delay(100);
}
