//ADDED FOR COMPATIBILITY WITH WIRING
extern "C" {
  #include <stdlib.h>
}

#include "Arduino.h"
#include "Messenger.h"

Messenger::Messenger() {
	token[0] = ' ';
	token[1] = 0;
	bufferLength = MESSENGERBUFFERSIZE;
  bufferLastIndex = MESSENGERBUFFERSIZE -1;
  reset();
}

void Messenger::reset() {
  bufferIndex = 0;
  messageState = 0;
  current = NULL;
  last = NULL;
  dumped = 1;
}

int Messenger::readInt() {
  if (next()) {
  	dumped = 1;
  	return atoi(current);
  }
  return 0;
}

uint8_t Messenger::next() {
  char * temppointer= NULL;
  switch (messageState) {
  case 0:
    return 0;
  case 1:
    temppointer = buffer;
    messageState = 2;
  default:
    if (dumped) current = strtok_r(temppointer,token,&last);
    if (current != NULL) {
    	dumped = 0;
    	return 1;
	  }
  }
  return 0;
}

uint8_t Messenger::available() {
  return next();
}


uint8_t Messenger::process(int serialByte) {
  messageState = 0;
  if (serialByte > 0) {
    switch (serialByte) {
      case 0:
      	break;
      case 10: // LF
        break;
      case 13: // CR
        buffer[bufferIndex]=0;
        reset();
        messageState = 1;
        current = buffer;
        break;
      default:
        buffer[bufferIndex]=serialByte;
        bufferIndex++;
        if (bufferIndex >= bufferLastIndex) reset();
    }
  }
  return messageState;
}
