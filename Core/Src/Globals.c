#include "Globals.h"
#include "stdbool.h"


// Declare global variables
volatile int speed = 50;
volatile int16_t AccData[3] = { 0 };
volatile float TempData = 0;
volatile int16_t GyroData[3] = { 0 };
volatile int16_t MagData[3] = { 0 };

volatile uint8_t RemoteBuffer = 0;
volatile uint8_t ibusIndex = 0;	// Current position in the ibus packet
volatile uint8_t ibusData[IBUS_BUFFSIZE] = { 0 };	// Ibus packet buffer
volatile bool ProcessRemoteBuffer = false;
