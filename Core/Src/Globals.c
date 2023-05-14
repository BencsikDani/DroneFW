#include "Globals.h"
#include "stdbool.h"


// Declare global variables
volatile int Thrust = 50;
volatile int Pitch = 0;
volatile int Roll = 0;
volatile int Yaw = 0;

int16_t AccData[3] = { 0 };
float TempData = 0;
int16_t GyroData[3] = { 0 };
int16_t MagData[3] = { 0 };

volatile uint8_t Uart2Buffer = 0;
volatile uint8_t IbusIndex = 0;	// Current position in the ibus packet
volatile uint8_t IbusPackageBuffer[IBUS_BUFFSIZE] = { 0 };	// Ibus packet buffer
volatile bool ProcessRemoteBuffer = false;
