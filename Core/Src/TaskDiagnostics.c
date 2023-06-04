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

void TaskDiagnostics(void const *argument)
{
	char UARTstr[512];
	uint8_t SPIdata[64];

	/* Infinite loop */
	while (1)
	{
		if(osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr, "Throttle: (%d) %d %d %d %d\r\n", Throttle, TIM3->CCR3, TIM3->CCR4, TIM3->CCR1, TIM3->CCR2);
			SPIdata[0] = (uint8_t)Throttle;
			SPIdata[1] = (uint8_t)TIM3->CCR3;
			SPIdata[2] = (uint8_t)TIM3->CCR4;
			SPIdata[3] = (uint8_t)TIM3->CCR1;
			SPIdata[4] = (uint8_t)TIM3->CCR2;

			sprintf(UARTstr, "%sYaw: %d\r\n", UARTstr, Yaw);
			SPIdata[5] = (uint8_t)Yaw;

			sprintf(UARTstr, "%sPitch: %d\r\n", UARTstr, Pitch);
			SPIdata[6] = (uint8_t)Pitch;

			sprintf(UARTstr, "%sRoll: %d\r\n", UARTstr, Roll);
			SPIdata[7] = (uint8_t)Roll;

			sprintf(UARTstr, "%sSWA: %d\r\n", UARTstr, SWA);
			SPIdata[8] = (uint8_t)SWA;

			sprintf(UARTstr, "%sSWB: %d\r\n", UARTstr, SWB);
			SPIdata[9] = (uint8_t)SWB;

			sprintf(UARTstr, "%sSWC: %d\r\n", UARTstr, SWC);
			SPIdata[10] = (uint8_t)SWC;

			sprintf(UARTstr, "%sSWD: %d\r\n", UARTstr, SWD);
			SPIdata[11] = (uint8_t)SWD;

			sprintf(UARTstr, "%sVRA: %d\r\n", UARTstr, VRA);
			SPIdata[12] = (uint8_t)VRA;

			sprintf(UARTstr, "%sVRB: %d\r\n", UARTstr, VRB);
			SPIdata[13] = (uint8_t)VRB;
		}
		osMutexRelease(RemoteDataMutexHandle);


		if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr,
					"%sTemp: %.4f\r\nAcc:  %1.4f ; %1.4f ; %1.4f\r\nGyro: %1.4f ; %1.4f ; %1.4f\r\n",
					UARTstr,
					TempData, AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2]);
			SPIdata[14] = (uint8_t)TempData;
			SPIdata[15] = (uint8_t)AccData[0];
			SPIdata[16] = (uint8_t)AccData[1];
			SPIdata[17] = (uint8_t)AccData[2];
			SPIdata[18] = (uint8_t)GyroData[0];
			SPIdata[19] = (uint8_t)GyroData[1];
			SPIdata[20] = (uint8_t)GyroData[2];

			sprintf(UARTstr,
					"%sBMP_Temp: %.4f\r\nBMP_Pres: %.4f\r\nBMP_Alt: %.4f\r\n",
					UARTstr,
					BMP_Temp, BMP_Pres, BMP_Alt);
			SPIdata[21] = (uint8_t)BMP_Temp;
			SPIdata[22] = (uint8_t)BMP_Pres;
			SPIdata[23] = (uint8_t)BMP_Alt;
		}
		osMutexRelease(ImuMutexHandle);


		if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr,
					"%sMAG_X_RAW: %.4f\r\nMAG_Y_RAW: %.4f\r\nMAG_Z_RAW: %.4f\r\ndir: %.4f\r\n",
					UARTstr,
					MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW, MAG_dir);
			SPIdata[24] = (uint8_t)MAG_X_RAW;
			SPIdata[25] = (uint8_t)MAG_Y_RAW;
			SPIdata[26] = (uint8_t)MAG_Z_RAW;
			SPIdata[27] = (uint8_t)MAG_dir;
		}
		osMutexRelease(MagnMutexHandle);


		if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr, "%sDistance: %.0f mm\r\n", UARTstr, Distance);
			SPIdata[28] = (uint8_t)Distance;
		}
		osMutexRelease(DistMutexHandle);


		if (osMutexWait(GpsDataMutexHandle, osWaitForever) == osOK)
		{
			sprintf(UARTstr, "%sGPS:\tLat -> %.4f %c\r\n\tLong -> %.4f %c\r\n\tFix -> %d\r\n\tNOS -> %d\r\n\tHDOP -> %.4f\r\n\tAlt -> %.4f %c\r\n",
					UARTstr,
					GPS.dec_latitude, GPS.ns, GPS.dec_longitude, GPS.ew, GPS.fix, GPS.num_of_satelites, GPS.horizontal_dilution_of_precision, GPS.mean_sea_level_altitude, GPS.altitude_unit);
			SPIdata[29] = (uint8_t)GPS.dec_latitude;
			SPIdata[30] = (uint8_t)GPS.ns;
			SPIdata[31] = (uint8_t)GPS.dec_longitude;
			SPIdata[32] = (uint8_t)GPS.ew;
			SPIdata[33] = (uint8_t)GPS.fix;
			SPIdata[34] = (uint8_t)GPS.num_of_satelites;
			SPIdata[35] = (uint8_t)GPS.horizontal_dilution_of_precision;
			SPIdata[36] = (uint8_t)GPS.mean_sea_level_altitude;
			SPIdata[37] = (uint8_t)GPS.altitude_unit;
		}
		osMutexRelease(GpsDataMutexHandle);


		sprintf(UARTstr, "%s\r\n\r\n", UARTstr);

		// Sending log info
		if (Diag)
		{
			HAL_UART_Transmit(&huart3, UARTstr, strlen(UARTstr), HAL_MAX_DELAY);
			HAL_SPI_Transmit(&hspi1, SPIdata, 64, HAL_MAX_DELAY);
		}

		//char tempstr[30];
		//sprintf(tempstr, "SPI data sent!\r\n");
		//HAL_UART_Transmit(&huart3, tempstr, strlen(tempstr), HAL_MAX_DELAY);

		osDelay(200);
	}
}
