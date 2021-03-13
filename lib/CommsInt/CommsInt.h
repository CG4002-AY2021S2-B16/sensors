#ifndef _COMMSINT_H_
#define _COMMSINT_H_
#include <Arduino.h>

extern boolean new_handshake_req;
extern boolean handshake_done;

void resetTimeOffset();
bool checkLivenessPacketRequired();
void prepareAES();
void handshakeResponse();
void livenessResponse();
void IMUdataResponse(int16_t x, int16_t y, int16_t z, int16_t pitch, int16_t yaw, int16_t roll);
void EMGdataResponse(float MAV, float RMS, float MNF);
void receiveData();

#endif