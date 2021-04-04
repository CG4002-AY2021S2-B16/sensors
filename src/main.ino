#include <Arduino.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "CommsInt.h"

#define EMG_SENSOR_MODE false // set true for EMG beetle

#define ONBOARD_LED 13
#define EMG_PIN A0
#define IMU_INT_PIN A1

#define ACCEL_RANGE 2 // +- 8g for range from +32768/-32767
#define GYRO_RANGE 2

#define RAW_TO_MS_2 (float)__SHRT_MAX__ * 9.81 * 8
#define RAW_TO_DEG_S_2 (float)__SHRT_MAX__ * 1000.

#define ZERO_MOTION_THRESHOLD 3 // threshold = value * 2m
#define ZERO_MOTION_DURATION 15  // duration = value * 64ms

#define GYRO_X_THRESHOLD 500

MPU6050 imuSensor;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
bool isMotionDetected;
bool isShiftDetected;

bool getIsShiftDetected(int16_t gyro_val);

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
    // Edit in values based on bluno number
  imuSensor.setXAccelOffset(-1270);
  imuSensor.setYAccelOffset(-6598);
  imuSensor.setZAccelOffset(974);
  imuSensor.setXGyroOffset(-46);
  imuSensor.setYGyroOffset(-17);
  imuSensor.setZGyroOffset(-25);

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

  Serial.begin(115200);
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
    if (checkLivenessPacketRequired()) {
      livenessResponse();
    } else if (EMG_SENSOR_MODE) {
      // <-- Add in EMGdataResponse code here -->
      EMGdataResponse(0.00, 314.26, -527.984231); // dummy values for testing
      delay(128); // There should not be more than a (small << 128) delay on integrated code, because delay also comes from EMG sampling.
    } else {
      imuSensor.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
      isShiftDetected = getIsShiftDetected(gyroX);

      if (isMotionDetected || isShiftDetected) {
        IMUdataResponse(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
        delay(20);
      }
    }
  }
}


bool getIsShiftDetected(int16_t gyro_val)
{
  return (gyro_val > GYRO_X_THRESHOLD || gyro_val < -GYRO_X_THRESHOLD);
}
