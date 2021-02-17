/**
 * To find out the fatigue of a user, we are using the following research papers:
 * https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6679263/
 * 
 * The papers uses the following values to determine muscular fatigue:
 * - Mean Absolute Value of sensor reading (MAV)
 * - Root Mean Square Value of sensor reading (RMS)
 * - Mean Frequency (MNF)
 * 
 * The indicators of muscle fatigue would be shown by the reduction of average
 * frequency, and the increment of MAV and RMS.
 */

#include "Arduino.h"
#include "EMG.h"
#include "arduinoFFT.h"

#define MAX_VOLTAGE 5.0
#define ANALOG_SIZE 1023.0

double vReal[EMG_SAMPLE_SIZE];
double vImag[EMG_SAMPLE_SIZE];
double freqArr[EMG_SAMPLE_SIZE / 2];

uint16_t sampleCount = 0;
double totalSensorVal = 0;
double totalSquaredSensorVal = 0;
double maxSensorVal = 0;

arduinoFFT FFT = arduinoFFT();

void readEMGData(uint16_t sensorReading)
{
  float actualSensorVal = convertSensorValue(sensorReading);

  vReal[sampleCount] = sensorReading;
  vImag[sampleCount] = 0;

  totalSensorVal += actualSensorVal;
  totalSquaredSensorVal += (actualSensorVal * actualSensorVal);
  maxSensorVal = (actualSensorVal > maxSensorVal) ? actualSensorVal : maxSensorVal;

  ++sampleCount;
}

void processEMG(double *maxSensorValue, double *meanSensorValue, double *rmsSensorValue, double *meanSensorFrequency)
{
  computeFFT();
  *meanSensorValue = totalSensorVal / (EMG_SAMPLE_SIZE * 1.0);
  *rmsSensorValue = sqrt(totalSquaredSensorVal / (EMG_SAMPLE_SIZE * 1.0));
  *maxSensorValue = maxSensorVal;

  resetValues();
}

void resetValues()
{
  maxSensorVal = 0;
  totalSensorVal = 0;
  totalSquaredSensorVal = 0;
  sampleCount = 0;
}

void setupFrequencyArr()
{
  for (int i = 0; i < EMG_SAMPLE_SIZE / 2; i++)
  {
    freqArr[i] = (EMG_SAMPLE_FREQUENCY * i) / (EMG_SAMPLE_SIZE * 1.0);
  }
}

float convertSensorValue(uint16_t sensorReading)
{
  return (sensorReading / ANALOG_SIZE) * MAX_VOLTAGE;
}

double computeFFT()
{
  FFT.Windowing(vReal, EMG_SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, EMG_SAMPLE_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, EMG_SAMPLE_SIZE);

  long freqAndPowerSpecSum = 0;
  long powerSpecSum = 0;

  // for (int i = 0; i < EMG_SAMPLE_SIZE / 2; i++)
  // {
  //   unsigned int powerSpec = vReal[i] * vReal[i];
  //   freqAndPowerSpecSum += powerSpec * freqArr[i];
  //   powerSpecSum += powerSpec;
  // }

  return 0;
  // double calculatedMAF = (freqAndPowerSpecSum * 1.0) / (powerSpecSum * 1.0);
  // return calculatedMAF;
}
