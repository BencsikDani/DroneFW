#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "Globals.h"

#include "Debug.h"

extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId RemoteBufferSemaphoreHandle;
extern osMutexId MagnMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId DistMutexHandle;
extern osMutexId GpsDataMutexHandle;

void DisassembleFloatIntoUint8s(float* n, uint8_t* array, int position)
{
	memcpy(array+position, n, sizeof(float));
}

void TaskDiagnostics(void const *argument)
{
	char UARTstr[512];
	uint8_t SpiIntData[64];
	uint8_t SpiFloatData1[64];
	uint8_t SpiFloatData2[64];

	SpiIntData[0] = (uint8_t)('i');
	SpiFloatData1[0] = (uint8_t)('f');
	SpiFloatData2[0] = (uint8_t)('g');

	/* Infinite loop */
	while (1)
	{
		if(osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr, "Throttle: (%d) %d %d %d %d\r\n", Throttle, TIM3->CCR3, TIM3->CCR4, TIM3->CCR1, TIM3->CCR2);
			SpiIntData[1] = (uint8_t)Throttle;
			SpiIntData[2] = (uint8_t)TIM3->CCR3;
			SpiIntData[3] = (uint8_t)TIM3->CCR4;
			SpiIntData[4] = (uint8_t)TIM3->CCR1;
			SpiIntData[5] = (uint8_t)TIM3->CCR2;

			sprintf(UARTstr, "%sYaw: %d\r\n", UARTstr, Yaw);
			SpiIntData[6] = (uint8_t)Yaw;

			sprintf(UARTstr, "%sPitch: %d\r\n", UARTstr, Pitch);
			SpiIntData[7] = (uint8_t)Pitch;

			sprintf(UARTstr, "%sRoll: %d\r\n", UARTstr, Roll);
			SpiIntData[8] = (uint8_t)Roll;

			sprintf(UARTstr, "%sSWA: %d\r\n", UARTstr, SWA);
			SpiIntData[9] = (uint8_t)SWA;

			sprintf(UARTstr, "%sSWB: %d\r\n", UARTstr, SWB);
			SpiIntData[10] = (uint8_t)SWB;

			sprintf(UARTstr, "%sSWC: %d\r\n", UARTstr, SWC);
			SpiIntData[11] = (uint8_t)SWC;

			sprintf(UARTstr, "%sSWD: %d\r\n", UARTstr, SWD);
			SpiIntData[12] = (uint8_t)SWD;

			sprintf(UARTstr, "%sVRA: %d\r\n", UARTstr, VRA);
			SpiIntData[13] = (uint8_t)VRA;

			sprintf(UARTstr, "%sVRB: %d\r\n", UARTstr, VRB);
			SpiIntData[14] = (uint8_t)VRB;
		}
		osMutexRelease(RemoteDataMutexHandle);


		if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr,
					"%sTemp: %.4f\r\nAcc:  %1.4f ; %1.4f ; %1.4f\r\nGyro: %1.4f ; %1.4f ; %1.4f\r\n",
					UARTstr,
					TempData,
					AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2]);
			DisassembleFloatIntoUint8s(&TempData, SpiFloatData1, 1);
			DisassembleFloatIntoUint8s(AccData, SpiFloatData1, 5);
			DisassembleFloatIntoUint8s(AccData+1, SpiFloatData1, 9);
			DisassembleFloatIntoUint8s(AccData+2, SpiFloatData1, 13);
			DisassembleFloatIntoUint8s(GyroData, SpiFloatData1, 17);
			DisassembleFloatIntoUint8s(GyroData+1, SpiFloatData1, 21);
			DisassembleFloatIntoUint8s(GyroData+2, SpiFloatData1, 25);

			sprintf(UARTstr,
					"%sBMP_Temp: %.4f\r\nBMP_Pres: %.4f\r\nBMP_Alt: %.4f\r\n",
					UARTstr,
					BMP_Temp, BMP_Pres, BMP_Alt);
			DisassembleFloatIntoUint8s(&BMP_Temp, SpiFloatData1, 29);
			DisassembleFloatIntoUint8s(&BMP_Pres, SpiFloatData1, 33);
			DisassembleFloatIntoUint8s(&BMP_Alt, SpiFloatData1, 37);
		}
		osMutexRelease(ImuMutexHandle);


		if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr,
					"%sMAG_X_RAW: %.4f\r\nMAG_Y_RAW: %.4f\r\nMAG_Z_RAW: %.4f\r\ndir: %.4f\r\n",
					UARTstr,
					MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW, MAG_dir);
			DisassembleFloatIntoUint8s(&MAG_X_RAW, SpiFloatData1, 41);
			DisassembleFloatIntoUint8s(&MAG_Y_RAW, SpiFloatData1, 45);
			DisassembleFloatIntoUint8s(&MAG_Z_RAW, SpiFloatData1, 49);
			DisassembleFloatIntoUint8s(&MAG_dir, SpiFloatData1, 53);
		}
		osMutexRelease(MagnMutexHandle);


		if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr, "%sDistance: %.0f mm\r\n", UARTstr, Distance);
			DisassembleFloatIntoUint8s(&Distance, SpiFloatData1, 57);
		}
		osMutexRelease(DistMutexHandle);


		if (osMutexWait(GpsDataMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr, "%sGPS:\tLat -> %.4f %c\r\n\tLong -> %.4f %c\r\n\tFix -> %d\r\n\tNOS -> %d\r\n\tHDOP -> %.4f\r\n\tAlt -> %.4f %c\r\n",
					UARTstr,
					GPS.dec_latitude, GPS.ns, GPS.dec_longitude, GPS.ew, GPS.fix, GPS.num_of_satelites, GPS.horizontal_dilution_of_precision, GPS.mean_sea_level_altitude, GPS.altitude_unit);
			DisassembleFloatIntoUint8s(&GPS.dec_latitude, SpiFloatData2, 1);
			SpiIntData[15] = (uint8_t)GPS.ns;
			DisassembleFloatIntoUint8s(&GPS.dec_longitude, SpiFloatData2, 5);
			SpiIntData[16] = (uint8_t)GPS.ew;
			SpiIntData[17] = (uint8_t)(GPS.fix & 0x000000ff);
			SpiIntData[18] = (uint8_t)(GPS.num_of_satelites & 0x000000ff);
			DisassembleFloatIntoUint8s(&GPS.horizontal_dilution_of_precision, SpiFloatData2, 9);
			DisassembleFloatIntoUint8s(&GPS.mean_sea_level_altitude, SpiFloatData2, 13);
			SpiIntData[19] = (uint8_t)GPS.altitude_unit;
		}
		osMutexRelease(GpsDataMutexHandle);


		sprintf(UARTstr, "%s\r\n\r\n", UARTstr);

		// Sending log info
		if (Diag)
		{
			HAL_UART_Transmit(&huart3, UARTstr, strlen(UARTstr), HAL_MAX_DELAY);
			HAL_SPI_Transmit(&hspi1, SpiIntData, 64, HAL_MAX_DELAY);
			osDelay(10);
			HAL_SPI_Transmit(&hspi1, SpiFloatData1, 64, HAL_MAX_DELAY);
			osDelay(10);
			HAL_SPI_Transmit(&hspi1, SpiFloatData2, 64, HAL_MAX_DELAY);
		}

		osDelay(500);
	}
}
