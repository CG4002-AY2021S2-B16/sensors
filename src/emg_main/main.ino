#include <Arduino.h>
#include "Wire.h"

#include "CommsInt.h"

#include "arduinoFFT.h"
#include "EMG.h"

#define ONBOARD_LED 13
#define EMG_PIN A0

double maxSensorReading;
double meanSensorReading;
double rmsSensorReading;
double meanSensorFreq;

void setup()
{
  // Initialize the i2c wire connection
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(EMG_PIN, INPUT);

  // Indicate the start of initializing
  digitalWrite(ONBOARD_LED, HIGH);
  Wire.begin();
  prepareAES();

  setupFrequencyArr();

  // Indicate the end of initializing
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(115200);
  delay(1000);
}

void loop()
{
  receiveData();
  readEMGData(analogRead(EMG_PIN));
  if (new_handshake_req)
  {
    handshakeResponse();
    resetTimeOffset();
    handshake_done = true;
    delay(500);
  }
  else if (handshake_done)
  {
    if (checkLivenessPacketRequired())
    {
      livenessResponse();
    }
    else if (sampleCount >= EMG_SAMPLE_SIZE)
    {
      processEMG(&maxSensorReading, &meanSensorReading, &rmsSensorReading, &meanSensorFreq);
      EMGdataResponse((float)meanSensorReading, (float)rmsSensorReading, (float)meanSensorFreq);
      delay(128); // There should not be more than a (small << 128) delay on integrated code, because delay also comes from EMG sampling.
    }
  }
}
