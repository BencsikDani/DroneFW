#ifndef GLOBALS_H
#define GLOBALS_H


#include "stdio.h"
#include "stdbool.h"
#include "IMU/MPU9250.h"
#include "IMU/BMP280.h"


// Declare global defines
#define IBUS_BUFFSIZE 32    // Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_MAXCHANNELS 6 // My TX only has 6 channels, no point in polling the rest

#define THROTTLE_CHANNEL 3
#define YAW_CHANNEL 4
#define PITCH_CHANNEL 2
#define ROLL_CHANNEL 1

#define MPU9250_SPI hspi2

// Declare global variables
extern uint16_t Thrust;
extern uint16_t Pitch;
extern uint16_t Roll;
extern uint16_t Yaw;

extern MPU9250_t MPU9250;
extern float AccData[3];
extern float TempData;
extern float GyroData[3];
extern int16_t MagData[3];

extern BMP280_t BMP280;
extern float BMP_Temp;
extern float BMP_Pres;
extern float BMP_Alt;

extern volatile uint8_t Uart2Buffer;
extern volatile uint8_t IbusIndex;	// Current position in the ibus packet
extern volatile uint8_t IbusPackageBuffer[IBUS_BUFFSIZE];	// Ibus packet buffer
extern volatile bool ProcessRemoteBuffer;


#endif /* GLOBALS_H */
