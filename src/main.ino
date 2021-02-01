#include <Arduino.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "CommsInt.h"

// CONSTANTS
#define X_GYRO_OFFSET 0
#define Y_GYRO_OFFSET 0
#define Z_GYRO_OFFSET 0
#define X_ACCEL_OFFSET 0
#define Y_ACCEL_OFFSET 0
#define Z_ACCEL_OFFSET 0

MPU6050 imuSensor;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

void setIMUSensorOffset()
{
  imuSensor.setXGyroOffset(X_GYRO_OFFSET);
  imuSensor.setYGyroOffset(Y_GYRO_OFFSET);
  imuSensor.setZGyroOffset(Z_GYRO_OFFSET);
  imuSensor.setXAccelOffset(X_ACCEL_OFFSET);
  imuSensor.setYAccelOffset(Y_ACCEL_OFFSET);
  imuSensor.setZAccelOffset(Z_ACCEL_OFFSET);
}

void setup()
{
  // Initialize the i2c wire connection
  Wire.begin();

  Serial.begin(115200);
  Serial.println("Initializing I2C devices...");
  imuSensor.initialize();

  Serial.println("Testing device connections...");

  if (!imuSensor.testConnection())
  {
    Serial.println("MPU6050 connection failed!");
  }

  Serial.println("MPU6050 connection successful");
}

void loop()
{
  imuSensor.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  Serial.print(accelX);
  Serial.print(" ");
  Serial.print(accelY);
  Serial.print(" ");
  Serial.print(accelZ);
  Serial.print(" ");
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print(gyroZ);
  Serial.print("\n");
}