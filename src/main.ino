#include <Arduino.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "CommsInt.h"

#define ONBOARD_LED 13
#define EMG_PIN A0

MPU6050 imuSensor;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

void setup()
{
  // Initialize the i2c wire connection
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(EMG_PIN, INPUT);

  // Indicate the start of initializing
  digitalWrite(ONBOARD_LED, HIGH);
  Wire.begin();

  prepareAES();

  imuSensor.initialize();
  if (!imuSensor.testConnection())
  {
    Serial.println("MPU6050 connection failed!");
  }
  else
  {
    imuSensor.CalibrateAccel(10);
    imuSensor.CalibrateGyro(10);
    digitalWrite(ONBOARD_LED, LOW);
  }

  Serial.begin(115200);
  delay(1000);
}

void loop()
{
  receiveData();

  if (new_handshake_req)
  {
    handshakeResponse();
    resetTimeOffset();
    handshake_done = true;
    delay(500);
  }
  else if (handshake_done)
  {
    imuSensor.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    // Add data smoothing/function to reduce noise here
    dataResponse(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  }

  delay(20);
}
