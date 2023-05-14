#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "Globals.h"

#include "Debug.h"

extern UART_HandleTypeDef huart5;
extern osSemaphoreId RemoteBufferSemHandle;
extern osSemaphoreId RemoteDataSemHandle;
extern osSemaphoreId ImuSemHandle;
extern osSemaphoreId MagnSemHandle;
extern osSemaphoreId DistSemHandle;
extern osSemaphoreId GpsSemHandle;

void TaskDiagnostics(void const *argument)
{
	char str[100];

	/* Infinite loop */
	while (1)
	{
		Log("Diag - RDSemEnter");
		if(osSemaphoreWait(RemoteDataSemHandle, osWaitForever) == osOK)
		{
			//Log("Diag - RDSemEntered");
			sprintf(str, "Thrust: %d\r\n", Thrust);

			Log("Diag - RDSemRelease");
			osSemaphoreRelease(RemoteDataSemHandle);
			//Log("Diag - RDSemReleased");
		}

		Log("Diag - ISemEnter");
		if (osSemaphoreWait(ImuSemHandle, osWaitForever) == osOK)
		{
			//Log("Diag - ISemEntered");
			sprintf(str,
					"%sTemp: %.4f\r\nAcc:  %5d ; %5d ; %5d\r\nGyro: %5d ; %5d ; %5d\r\nMagn: %5d ; %5d ; %5d\r\n\r\n",
					str,
					TempData, AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2], MagData[0],
					MagData[1], MagData[2]);

			Log("Diag - ISemRelease");
			osSemaphoreRelease(ImuSemHandle);
			//Log("Diag - ISemReleased");
		}

		// Sending UART log info
		HAL_UART_Transmit(&huart5, str, strlen(str), HAL_MAX_DELAY);

		osDelay(100);
	}
}
