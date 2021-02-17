#ifndef _EMG_H_
#define _EMG_H_

#define EMG_SAMPLE_SIZE 128
#define EMG_SAMPLE_FREQUENCY 1000

extern double vReal[EMG_SAMPLE_SIZE];
extern double vImag[EMG_SAMPLE_SIZE];
extern uint16_t sampleCount;

void readEMGData(uint16_t sensorReading);
void processEMG(double *maxSensorValue, double *meanSensorValue, double *rmsSensorValue, double *meanSensorFrequency);
void resetValues();
void setupFrequencyArr();
float convertSensorValue(uint16_t sensorReading);
double computeFFT();
#endif