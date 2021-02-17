#include <Arduino.h>
#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "CommsInt.h"

#include "arduinoFFT.h"
#include "EMG.h"

#define ONBOARD_LED 13
#define EMG_PIN A0
#define IMU_INT_PIN A1

#define ACCEL_RANGE 2 // +- 8g for range from +32768/-32767
#define GYRO_RANGE 2

#define RAW_TO_MS_2 (float)__SHRT_MAX__ * 9.81 * 8
#define RAW_TO_DEG_S_2 (float)__SHRT_MAX__ * 1000.

#define ZERO_MOTION_THRESHOLD 8 // threshold = value * 2mg
#define ZERO_MOTION_DURATION 5  // duration = value * 64ms

MPU6050 imuSensor;

int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
double maxSensorReading;
double meanSensorReading;
double rmsSensorReading;
double meanSensorFreq;

bool isMotionDetected;

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
  if (imuSensor.testConnection())
  {
    // Calibrate IMU with 600 readings
    imuSensor.CalibrateAccel(6);
    imuSensor.CalibrateGyro(6);
    // Set IMU range
    imuSensor.setFullScaleAccelRange(ACCEL_RANGE);
    imuSensor.setFullScaleGyroRange(GYRO_RANGE);
    // Detection of Zero Motion
    imuSensor.setZeroMotionDetectionThreshold(ZERO_MOTION_THRESHOLD);
    imuSensor.setZeroMotionDetectionDuration(ZERO_MOTION_DURATION);

    setupFrequencyArr();

    // Indicate the end of initializing
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
}

void loop()
{
  isMotionDetected = !imuSensor.getZeroMotionDetected();
  imuSensor.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  readEMGData(analogRead(EMG_PIN));
  if (sampleCount >= EMG_SAMPLE_SIZE)
  {
    processEMG(&maxSensorReading, &meanSensorReading, &rmsSensorReading, &meanSensorFreq);
    Serial.print("maxSensorVal:");
    Serial.print(maxSensorReading);
    Serial.print(' ');
    Serial.print("meanSensorVal:");
    Serial.print(meanSensorReading);
    Serial.print(' ');
    Serial.print("rmsSensorVal:");
    Serial.print(rmsSensorReading);
    Serial.print(' ');
    Serial.print("meanSensorFreq:");
    Serial.println(meanSensorFreq);
  }

  if (isMotionDetected)
  {
    Serial.print("Accel_X:");
    Serial.print(accelX / RAW_TO_MS_2);
    Serial.print(' ');
    Serial.print("Accel_Y:");
    Serial.print(accelY / RAW_TO_MS_2);
    Serial.print(' ');
    Serial.print("Accel_Z:");
    Serial.print(accelZ / RAW_TO_MS_2);
    Serial.print(' ');
    Serial.print("Gyro_X:");
    Serial.print(gyroX / RAW_TO_DEG_S_2);
    Serial.print(' ');
    Serial.print("Gyro_Y:");
    Serial.print(gyroY / RAW_TO_DEG_S_2);
    Serial.print(' ');
    Serial.print("Gyro_Z:");
    Serial.println(gyroZ / RAW_TO_DEG_S_2);
  }
}
