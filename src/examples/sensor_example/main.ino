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

  Serial.begin(115200);

  imuSensor.initialize();
  imuSensor.CalibrateAccel(6);
  imuSensor.CalibrateGyro(6);
  imuSensor.setFullScaleAccelRange(2);
  imuSensor.setFullScaleGyroRange(2);
  digitalWrite(ONBOARD_LED, LOW);
}

void loop()
{
  imuSensor.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  Serial.print(accelX);
  Serial.print(' ');
  Serial.print(accelY);
  Serial.print(' ');
  Serial.print(accelZ);
  Serial.print(' ');
  Serial.print(gyroX);
  Serial.print(' ');
  Serial.print(gyroY);
  Serial.print(' ');
  Serial.println(gyroZ);
}