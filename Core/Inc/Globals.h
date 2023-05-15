#include "stdio.h"
#include "stdbool.h"
#include "MPU9250.h"


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
extern volatile int Thrust;
extern volatile int Pitch;
extern volatile int Roll;
extern volatile int Yaw;

extern float AccData[3];
extern float TempData;
extern float GyroData[3];
extern int16_t MagData[3];

extern uint8_t Uart2Buffer;
extern volatile uint8_t IbusIndex;	// Current position in the ibus packet
extern volatile uint8_t IbusPackageBuffer[IBUS_BUFFSIZE];	// Ibus packet buffer
extern volatile bool ProcessRemoteBuffer;

extern MPU9250_t MPU9250;

#endif /* GLOBALS_H */
