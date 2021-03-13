#ifndef _COMMSINT_H_
#define _COMMSINT_H_
#include <Arduino.h>

extern uint8_t receivedChar;
extern boolean new_handshake_req;
extern boolean handshake_done;
extern const byte AESKeyStageOne[];
extern const byte AESKeyStageTwo[];

void resetTimeOffset();
bool checkLivenessPacketRequired();
uint32_t calculateTimestamp();
void clearSendBuffer();
void setChecksum();
void prepareAES();
uint8_t* addIMUDataToBuffer(uint8_t* next, uint16_t x, uint16_t y, uint16_t z, uint16_t pitch, uint16_t yaw, uint16_t roll);
void handshakeResponse();
void livenessResponse();
void IMUdataResponse(int16_t x, int16_t y, int16_t z, int16_t pitch, int16_t yaw, int16_t roll);
void EMGdataResponse(float MAV, float RMS, float MNF);
void receiveData();

#endif