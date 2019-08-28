/*  The Port Manipulation info:
 *
 *  A B C D G H L
 *  0 1 2 3 4 5 6
 *
 *  ports array index: 5 ,3 ,3 ,3 ,3 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,2 ,2 ,2 ,2 ,2 ,2 ,2 ,2 ,3 ,4 ,4 ,4 ,6 ,6 ,6 ,6 ,6 ,6 ,6 ,6 ,1 ,1 ,1
 *  pin name:          H0,D3,D2,D1,D0,A0,A1,A2,A3,A4,A5,A6,A7,C7,C6,C5,C4,C3,C2,C1,C0,D7,G2,G1,G0,L7,L6,L5,L4,L3,L2,L1,L0,B3,B2,B1
 *  port bit index:     0, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1, 0, 7, 2, 1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 3, 2, 1
 *  physical pin:      17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52
 */

#include <Arduino.h>
#include <Messenger.h>
Messenger message = Messenger();

#define timerInterval 1000000 // 1.000.000 microseconds in a second

#define totalPins 36

uint8_t rgbDuty[totalPins];
unsigned long periodSize = 10000; // 100Hz
unsigned long lastPeriodStart = micros();

static volatile uint8_t *portsArray[] = { &PORTA, &PORTB, &PORTC, &PORTD, &PORTG, &PORTH, &PORTL };
uint8_t pinRegs[7] = { 0, 0, 0, 0, 0, 0, 0 };

                                    //17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52
const uint8_t portIndex[totalPins] = { 5, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 3, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 1, 1, 1 };
const uint8_t bitIndex[totalPins]  = { 0, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 7, 6, 5, 4, 3, 2, 1, 0, 7, 2, 1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 3, 2, 1 };

void setup() {
  Serial.begin(115200);

  // Clear duty array
  for (uint8_t i = 0; i < totalPins; i++) {
    rgbDuty[i] = 0;
  }

  // Setup ports
  DDRA = DDRA | B11111111;
  DDRB = DDRB | B00001110;
  DDRC = DDRC | B11111111;
  DDRD = DDRD | B10001111;
  DDRG = DDRG | B00000111;
  DDRH = DDRH | B00000001;
  DDRL = DDRL | B11111111;
}

void loop() {
  unsigned long timerCounter = micros() - lastPeriodStart;
  if (timerCounter >= periodSize) { // Turn on the pins that are in use when the counter is bigger than our period
    PORTA = pinRegs[0];
    PORTB = pinRegs[1];
    PORTC = pinRegs[2];
    PORTD = pinRegs[3];
    PORTG = pinRegs[4];
    PORTH = pinRegs[5];
    PORTL = pinRegs[6];
    lastPeriodStart = micros();
  } else {
    for (uint8_t i = 0; i < totalPins; i++) {
      if (rgbDuty[i] > 0 && timerCounter >= ((periodSize * rgbDuty[i]) / 100)) { // Turn off pin when the counter is bigger than its duty
        *(portsArray[portIndex[i]]) &= ~(1 << bitIndex[i]);
      }
    }
  }

  uint8_t messCount = 0;
  uint32_t messArray[5];
  while (Serial.available()) { // Maybe I should clear the buffer after this..
    if (message.process(Serial.read())) {
      while (message.available() && messCount < 5) {
        messArray[messCount] = message.readInt();
        messCount++;
      }
    }
  }
  if (messCount == 5 && messArray[0] >= 0 && messArray[0] < 12 && messArray[1] > 0) { // Right amount of messages, LEDstrip number in range and frequency higher than 0
    periodSize = timerInterval / messArray[1]; // countsPerSecond / desiredFrequency
    messArray[0] *= 3; // Pin calculation step I
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t pinIndex = messArray[0] + i; // Pin calculation step II
      rgbDuty[pinIndex] = messArray[i + 2] > 0 ? messArray[i + 2] < 100 ? messArray[i + 2] : 100 : 0; // Duty in range
      if (messArray[i + 2] > 0) {
        pinRegs[portIndex[pinIndex]] |= (1 << bitIndex[pinIndex]); // Pin in use, so turn it on in our private registry
      } else {
        pinRegs[portIndex[pinIndex]] &= ~(1 << bitIndex[pinIndex]); // Pin not in use, so turn it off in our private registry
      }
    }
  }
}
