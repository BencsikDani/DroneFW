#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "Globals.h"

#include "Debug.h"

extern UART_HandleTypeDef huart5;
extern osMutexId MagnMutexHandle;
extern osMutexId RemoteBufferMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId DistMutexHandle;
extern osMutexId GpsMutexHandle;

void TaskDiagnostics(void const *argument)
{
	char str[100];

	/* Infinite loop */
	while (1)
	{
		//Log("Diag - RDMutEnter");
		if(osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			//Log("Diag - RDMutEntered");
			sprintf(str, "Thrust: %d\r\n", Thrust);

			//Log("Diag - RDMutRelease");
			osMutexRelease(RemoteDataMutexHandle);
			//Log("Diag - RDMutReleased");
		}

		//Log("Diag - IMutEnter");
		if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			//Log("Diag - IMutEntered");
			sprintf(str,
					"%sTemp: %.4f\r\nAcc:  %5d ; %5d ; %5d\r\nGyro: %5d ; %5d ; %5d\r\nMagn: %5d ; %5d ; %5d\r\n\r\n",
					str,
					TempData, AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2], MagData[0],
					MagData[1], MagData[2]);

			//Log("Diag - IMutRelease");
			osMutexRelease(ImuMutexHandle);
			//Log("Diag - IMutReleased");
		}

		// Sending UART log info
		HAL_UART_Transmit(&huart5, str, strlen(str), HAL_MAX_DELAY);

		osDelay(100);
	}
}
