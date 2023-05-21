#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "Globals.h"

#include "Debug.h"

extern UART_HandleTypeDef huart3;
extern osSemaphoreId RemoteBufferSemaphoreHandle;
extern osMutexId MagnMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId DistMutexHandle;
extern osMutexId GpsMutexHandle;

void TaskDiagnostics(void const *argument)
{
	char str[300];

	/* Infinite loop */
	while (1)
	{
		//Log("Diag - RDMutEnter");
		if(osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			//Log("Diag - RDMutEntered");
			sprintf(str, "Throttle: (%d) %d %d %d %d\r\n", Throttle, TIM3->CCR3, TIM3->CCR4, TIM3->CCR1, TIM3->CCR2);
			sprintf(str, "%sYaw: %d\r\n", str, Yaw);
			sprintf(str, "%sPitch: %d\r\n", str, Pitch);
			sprintf(str, "%sRoll: %d\r\n", str, Roll);
			sprintf(str, "%sSWA: %d\r\n", str, SWA);
			sprintf(str, "%sSWB: %d\r\n", str, SWB);
			sprintf(str, "%sSWC: %d\r\n", str, SWC);
			sprintf(str, "%sSWD: %d\r\n", str, SWD);
			sprintf(str, "%sVRA: %d\r\n", str, VRA);
			sprintf(str, "%sVRB: %d\r\n", str, VRB);

			//Log("Diag - RDMutRelease");
			//osMutexRelease(RemoteDataMutexHandle);
			//Log("Diag - RDMutReleased");
		}
		osMutexRelease(RemoteDataMutexHandle);

		//Log("Diag - IMutEnter");
		if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			//Log("Diag - IMutEntered");
			sprintf(str,
					"%sTemp: %.4f\r\nAcc:  %1.4f ; %1.4f ; %1.4f\r\nGyro: %1.4f ; %1.4f ; %1.4f\r\n",
					str,
					TempData, AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2]);

			sprintf(str,
					"%sBMP_Temp: %.4f\r\nBMP_Pres: %.4f\r\nBMP_Alt: %.4f\r\n",
					str,
					BMP_Temp, BMP_Pres, BMP_Alt);

			//Log("Diag - IMutRelease");
			//osMutexRelease(ImuMutexHandle);
			//Log("Diag - IMutReleased");
		}
		osMutexRelease(ImuMutexHandle);

		if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
		{
			sprintf(str,
					"%sMAG_X_RAW: %.4f\r\nMAG_Y_RAW: %.4f\r\nMAG_Z_RAW: %.4f\r\ndir: %.4f\r\n",
					str,
					MAG_X_RAW, MAG_Y_RAW, MAG_Z_RAW, MAG_dir);
		}
		osMutexRelease(MagnMutexHandle);

		if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
		{
			sprintf(str, "%sDistance: %.0f mm\r\n", str, Distance);
		}
		osMutexRelease(DistMutexHandle);

		// Sending UART log info
		sprintf(str, "%s\r\n", str);
		HAL_UART_Transmit(&huart3, str, strlen(str), HAL_MAX_DELAY);

		osDelay(100);
	}
}
