#ifndef _COMMSINT_H_
#define _COMMSINT_H_
#include <Arduino.h>

extern char receivedChar;
extern boolean new_handshake_req;
extern boolean handshake_done;

void clearSendBuffer();
void fillChecksum();
char *addIntToBuffer(char *start, uint16_t x);
char *addDataToBuffer(char *next, uint16_t x, uint16_t y, uint16_t z, uint16_t yaw, uint16_t pitch, uint16_t roll);
void handshakeResponse();
void dataResponse();
void receiveData();

#endif