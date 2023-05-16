#include <GY-91/BMP280.h>
#include <GY-91/MPU9250.h>
#include <GY-271/HMC5883L.h>
#include "math.h"
#include "Globals.h"

#include "stdbool.h"


// Declare global variables
uint16_t Thrust = 50;
uint16_t Pitch = 0;
uint16_t Roll = 0;
uint16_t Yaw = 0;

MPU9250_t MPU9250;
float AccData[3] = { 0 };
float TempData = 0;
float GyroData[3] = { 0 };
int16_t MagData[3] = { 0 };

BMP280_t BMP280;
float BMP_Temp = 0;
float BMP_Pres = 0;
float BMP_Alt = 0;

float mG_per_LSB;
Vector v;
int xOffset, yOffset;
float declination = 5.57;
float MAG_X_RAW = 0;
float MAG_Y_RAW = 0;
float MAG_Z_RAW = 0;
float MAG_X_NORM = 0;
float MAG_Y_NORM = 0;
float MAG_Z_NORM = 0;
float magnitude = 0;
float MAG_dir = 0;

volatile uint8_t Uart2Buffer = 0;
volatile uint8_t IbusIndex = 0;	// Current position in the ibus packet
volatile uint8_t IbusPackageBuffer[IBUS_BUFFSIZE] = { 0 };	// Ibus packet buffer
volatile bool ProcessRemoteBuffer = false;




