#ifndef Messenger_h
#define Messenger_h
#define MESSENGERBUFFERSIZE 64

#include <inttypes.h>

class Messenger {
  public:
    Messenger();
    int readInt();
    uint8_t process(int serialByte);
    uint8_t available();

  private:
    uint8_t next();
    void reset();

    uint8_t messageState;

    char* current; // Pointer to current data
    char* last;

    char token[2];
    uint8_t dumped;

    uint8_t bufferIndex; // Index where to write the data
    char buffer[MESSENGERBUFFERSIZE]; // Buffer that holds the data
    uint8_t bufferLength; // The length of the buffer (defaults to 64)
    uint8_t bufferLastIndex; // The last index of the buffer
};

#endif
