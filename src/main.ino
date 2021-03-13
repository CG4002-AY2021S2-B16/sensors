#include <Arduino.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "CommsInt.h"

#define ONBOARD_LED 13
#define EMG_PIN A0
#define IMU_INT_PIN A1

#define ACCEL_RANGE 2 // +- 8g for range from +32768/-32767
#define GYRO_RANGE 2

#define RAW_TO_MS_2 (float)__SHRT_MAX__ * 9.81 * 8
#define RAW_TO_DEG_S_2 (float)__SHRT_MAX__ * 1000.

#define ZERO_MOTION_THRESHOLD 8 // threshold = value * 2m
#define ZERO_MOTION_DURATION 30  // duration = value * 64ms

MPU6050 imuSensor;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
bool isMotionDetected;

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
  if (imuSensor.testConnection())
  {
    // Calibrate IMU with 600 readings
    imuSensor.CalibrateAccel(6);
    imuSensor.CalibrateGyro(6);
    // Set IMU range
    imuSensor.setFullScaleAccelRange(ACCEL_RANGE);
    imuSensor.setFullScaleGyroRange(GYRO_RANGE);
    // Set zero motion detection
    imuSensor.setZeroMotionDetectionThreshold(ZERO_MOTION_THRESHOLD);
    imuSensor.setZeroMotionDetectionDuration(ZERO_MOTION_DURATION);

    digitalWrite(ONBOARD_LED, LOW);
  }
  else // Error connecting with MPU
  {
    for (;;)
    {
      digitalWrite(ONBOARD_LED, HIGH);
      delay(500);
      digitalWrite(ONBOARD_LED, LOW);
      delay(500);
    }
  }

  delay(1000);
}

void loop()
{
  isMotionDetected = !imuSensor.getZeroMotionDetected();
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
    if (isMotionDetected) {
      dataResponse(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    } else if (checkLivenessPacketRequired()) {
      livenessResponse();
    }
  }

  delay(20);
}
