#ifndef _COMMSINT_H_
#define _COMMSINT_H_
#include <Arduino.h>

extern char receivedChar;
extern boolean new_handshake_req;
extern boolean handshake_done;
extern boolean muscle_sensor_active;

void resetTimeOffset();
uint32_t calculateTimestamp();
void clearSendBuffer();
void setChecksum();
uint8_t* addIntToBuffer(uint8_t* start, uint16_t x);
uint8_t* addLongToBuffer(uint8_t* start, uint32_t x);
uint8_t* addIMUDataToBuffer(uint8_t* next, uint16_t x, uint16_t y, uint16_t z, uint16_t yaw, uint16_t pitch, uint16_t roll);
uint8_t* addMuscleSensorDataToBuffer(uint8_t* next, uint16_t ms_val);
uint8_t* setDataPacketTypeToBuffer(uint8_t* next);
uint8_t* setAckPacketTypeToBuffer(uint8_t* next);
void handshakeResponse();
void dataResponse(int16_t X, int16_t Y, int16_t Z, int16_t pitch, int16_t yaw, int16_t roll);
void receiveData();

#endif