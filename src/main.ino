#include <Arduino.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "CommsInt.h"

// CONSTANTS bluno 3
#define X_GYRO_OFFSET -792 //-976
#define Y_GYRO_OFFSET -2441 // -6527
#define Z_GYRO_OFFSET 1544 //988
#define X_ACCEL_OFFSET 143 //-52
#define Y_ACCEL_OFFSET 5 // -17
#define Z_ACCEL_OFFSET 33 //-24

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

  prepareAES();
  Serial.begin(115200);
  delay(1000);

  imuSensor.initialize();
  if (!imuSensor.testConnection())
  {
    Serial.println("MPU6050 connection failed!");
  }
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
    dataResponse(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  }

  delay(20);
}
