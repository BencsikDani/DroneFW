/*
 * MPU9250.c
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */

#include "main.h"
#include "stdbool.h"
#include "MPU9250.h"

extern SPI_HandleTypeDef hspi2;

#ifndef UTIL_MPU9250_CONFIG_H_
#define UTIL_MPU9250_CONFIG_H_

#define MPU9250_SPI			hspi2
#define	MPU9250_CS_GPIO		IMU_CSIMU_GPIO_Port
#define	MPU9250_CS_PIN		IMU_CSIMU_Pin
#endif /* UTIL_MPU9250_CONFIG_H_ */

const uint8_t MULTIPLEBYTE_CMD = 0x40;
const uint8_t DUMMY_BYTE = 0x00;

const uint8_t _address = 0b11010000;
// 400 kHz
const uint32_t _i2cRate = 400000;

// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t DLPF_184 = 0x01;
const uint8_t DLPF_92 = 0x02;
const uint8_t DLPF_41 = 0x03;
const uint8_t DLPF_20 = 0x04;
const uint8_t DLPF_10 = 0x05;
const uint8_t DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;

// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;

static uint8_t _buffer[96 - 59 + 1];
static uint8_t _mag_adjust[3];

int16_t AccBias[3] =
{ 0 };
int16_t GyroBias[3] =
{ 0 };
int16_t MagBias[3] =
{ 0 };

static inline void MPU9250_Activate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t address, uint8_t data)
{
	MPU9250_Activate();

	uint8_t control = address & ~0b10000000;
	HAL_SPI_Transmit(&MPU9250_SPI, &control, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);

	MPU9250_Deactivate();
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t address, uint8_t count, uint8_t *buffer)
{
	MPU9250_Activate();

	uint8_t control = address | 0b10000000;
	HAL_SPI_Transmit(&MPU9250_SPI, &control, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&MPU9250_SPI, buffer, count, HAL_MAX_DELAY);

	MPU9250_Deactivate();
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG, subAddress);

	// store the data for write
	writeRegister(I2C_SLV0_DO, data);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t) 1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG, subAddress);

	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00, count, dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI()
{
	// read the WHO AM I register
	readRegisters(WHO_AM_I, 1, _buffer);

	// return the register value
	return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963()
{
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer);
	// return the register value
	return _buffer[0];
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init()
{
	// check the WHO AM I byte, expected value is 0x71 (decimal 113)
	/*uint8_t who = whoAmI();
	 if(who != 0x71)
	 return 1;*/

	// Temporary variable, which holds the read configs
	uint8_t conf = 0;

	// reset the MPU9250
	writeRegister(PWR_MGMNT_1, PWR_RESET);
	// wait for MPU-9250 to come back up
	HAL_Delay(100);

	conf |= CLOCK_SEL_PLL; // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	writeRegister(PWR_MGMNT_1, conf);

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2, 0b00000000);

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set gyro and temp sensor bandwidth to 184 and 188 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 001; this sets the sample rate at 1 kHz for both
	// Maximum delay is 2.9 ms
	writeRegister(CONFIG, DLPF_184);

	//Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeRegister(SMPDIV, 0x00);

	// Set gyroscope full scale range
	readRegisters(GYRO_CONFIG, sizeof(conf), &conf); // get current GYRO_CONFIG register value
	// conf &= ~0b11100000; // Clear self-test bits [7:5]
	conf &= ~0b00000010; // Clear Fchoice bits [1:0]
	conf &= ~0b00011000; // Clear FS bits [4:3]
	conf |= GYRO_FS_SEL_250DPS; // Set FS bits [4:3]
	// conf =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeRegister(GYRO_CONFIG, conf); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	readRegisters(ACCEL_CONFIG, sizeof(conf), &conf); // get current ACCEL_CONFIG register value
	// conf &= ~0b11100000; // Clear self-test bits [7:5]
	conf &= ~0b00011000;  // Clear FS bits [4:3]
	conf |= ACCEL_FS_SEL_4G; // Set full scale range for the accelerometer
	writeRegister(ACCEL_CONFIG, conf);

	// Set accelerometer sample rate configuration
	readRegisters(ACCEL_CONFIG2, sizeof(conf), &conf); // get current ACCEL_CONFIG2 register value
	conf &= ~0b00001111; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	writeRegister(ACCEL_CONFIG2, conf);

	// enable I2C master mode
	readRegisters(USER_CTRL, sizeof(conf), &conf);
	conf &= 0b00000000;
	conf |= I2C_MST_EN;
	writeRegister(USER_CTRL, conf);

	// set the I2C bus speed to 400 kHz
	readRegisters(I2C_MST_CTRL, sizeof(conf), &conf);
	conf &= 0b00000000;
	conf |= I2C_MST_CLK;
	writeRegister(I2C_MST_CTRL, conf);

	HAL_Delay(100);

	// successful init, return 0
	return 0;

	/*
	 // set AK8963 to Power Down
	 writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
	 // reset the AK8963
	 writeAK8963Register(AK8963_CNTL2,AK8963_RESET);

	 // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	 if( whoAmIAK8963() != 0x48 )
	 {
	 return 1;
	 }

	 // get the magnetometer calibration
	 // set AK8963 to Power Down
	 writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	 HAL_Delay(100); // long wait between AK8963 mode changes

	 // set AK8963 to FUSE ROM access
	 writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM);

	 // long wait between AK8963 mode changes
	 HAL_Delay(100);

	 // read the AK8963 ASA registers and compute magnetometer scale factors
	 readAK8963Registers(AK8963_ASA, 3, _mag_adjust);

	 // set AK8963 to Power Down
	 writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);

	 // long wait between AK8963 mode changes
	 HAL_Delay(100);

	 // set AK8963 to 16 bit resolution, 100 Hz update rate
	 writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2);

	 // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	 readAK8963Registers(AK8963_HXL,7,_buffer);
	 */
}

void MPU9250_Calibrate()
{
	// Calculate average biases at resting point
	int avgCount = 10;

	int16_t AccBiasTemp[3] =
	{ 0 };
	int16_t GyroBiasTemp[3] =
	{ 0 };
	int16_t MagBiasTemp[3] =
	{ 0 };
	float TempTemp;
	int32_t AccBiasSum[3] =
	{ 0 };
	int32_t GyroBiasSum[3] =
	{ 0 };
	int32_t MagBiasSum[3] =
	{ 0 };

	// Calculate bias values
	for (int i = 0; i < avgCount; i++)
	{
		MPU9250_GetData(AccBiasTemp, &TempTemp, GyroBiasTemp, MagBiasTemp,
				true);

		for (int j = 0; j < 3; j++)
		{
			AccBiasSum[j] += AccBiasTemp[j];
			GyroBiasSum[j] += GyroBiasTemp[j];
			MagBiasSum[j] += MagBiasTemp[j];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		AccBias[i] = AccBiasSum[i] / avgCount;
		GyroBias[i] = GyroBiasSum[i] / avgCount;
		MagBias[i] = MagBiasSum[i] / avgCount;
	}
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
	writeRegister(ACCEL_CONFIG2, bandwidth);
	writeRegister(CONFIG, bandwidth);
}

/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(SMPDIV, 19);

	if (srd > 9)
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer);

	}
	else
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer);
	}

	writeRegister(SMPDIV, srd);
}

/* read the data, each argiment should point to a array for x, y, and x */
void MPU9250_GetData(int16_t *AccData, float *TempData, int16_t *GyroData,
		int16_t *MagData, bool calibrate)
{
	uint8_t buffer[96 - 59 + 1] =
	{ 0 };
	// grab the data from the MPU9250 (Reg 59-96)
	readRegisters(ACCEL_OUT, sizeof(buffer), buffer);

	// combine into 16 bit values
	// Accelerometer - Reg 69-64
	AccData[0] = (((int16_t) buffer[0]) << 8) + buffer[1];
	AccData[1] = (((int16_t) buffer[2]) << 8) + buffer[3];
	AccData[2] = (((int16_t) buffer[4]) << 8) + buffer[5];
	if (!calibrate)
	{
		for (int i = 0; i < 3; i++)
		{
			AccData[i] = (i != 2) ? AccData[i] - AccBias[i] : AccData[i];
			AccData[i] /= 48;
		}
	}

	// Temperature - Reg 65-66
	TempData[0] = (float) ((((int16_t) buffer[6]) << 8) + buffer[7]);
	TempData[0] = TempData[0] / 333.87 + 21;

	// Gyroscope - Reg 67-72
	GyroData[0] = (((int16_t) buffer[8]) << 8) + buffer[9];
	GyroData[1] = (((int16_t) buffer[10]) << 8) + buffer[11];
	GyroData[2] = (((int16_t) buffer[12]) << 8) + buffer[13];
	if (!calibrate)
	{
		for (int i = 0; i < 3; i++)
		{
			GyroData[i] -= GyroBias[i];
			GyroData[i] *= 250.0 / 32768.0;
		}
	}

	// External - Reg 73-96
	int16_t magx = (((int16_t) buffer[15]) << 8) | buffer[14];
	int16_t magy = (((int16_t) buffer[17]) << 8) | buffer[16];
	int16_t magz = (((int16_t) buffer[19]) << 8) | buffer[18];

	MagData[0] = (int16_t) ((float) magx
			* ((float) (_mag_adjust[0] - 128) / 256.0f + 1.0f));
	MagData[1] = (int16_t) ((float) magy
			* ((float) (_mag_adjust[1] - 128) / 256.0f + 1.0f));
	MagData[2] = (int16_t) ((float) magz
			* ((float) (_mag_adjust[2] - 128) / 256.0f + 1.0f));

	return;
}
