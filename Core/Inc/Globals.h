#include "stdio.h"
#include "stdbool.h"


#ifndef GLOBALS_H
#define GLOBALS_H

// Declare global defines
#define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 6 // My TX only has 6 channels, no point in polling the rest

#define THROTTLE_CHANNEL 3
#define YAW_CHANNEL 4
#define PITCH_CHANNEL 2
#define ROLL_CHANNEL 1

#define MPU9250_SPI hspi2

// Declare global variables
extern volatile int speed;
extern volatile int16_t AccData[3];
extern volatile float TempData;
extern volatile int16_t GyroData[3];
extern volatile int16_t MagData[3];

extern volatile uint8_t RemoteBuffer;
extern volatile uint8_t ibusIndex;	// Current position in the ibus packet
extern volatile uint8_t ibusData[IBUS_BUFFSIZE];	// Ibus packet buffer
extern volatile bool ProcessRemoteBuffer;

#endif /* GLOBALS_H */
