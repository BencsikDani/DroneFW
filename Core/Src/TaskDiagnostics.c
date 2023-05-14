#include "main.h"
#include "cmsis_os.h"

#include "Globals.h"

extern UART_HandleTypeDef huart5;
extern osSemaphoreId BinarySemHandle;

/* USER CODE BEGIN Header_RunTaskDiagnostics */
/**
* @brief Function implementing the TaskDiagnostics thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunTaskDiagnostics */
void RunTaskDiagnostics(void const * argument)
{
  /* USER CODE BEGIN RunTaskDiagnostics */
	char str[100];

	/* Infinite loop */
	while (1)
	{
		if (osSemaphoreWait(BinarySemHandle, osWaitForever) == osOK)
		{
			//HAL_UART_Transmit(&huart5, "DEB2\r\n", strlen("DEB2\r\n"), HAL_MAX_DELAY);

			// Setting up UART log info
			sprintf(str,
					"Speed: %d\r\nTemp: %.4f\r\nAcc:  %5d ; %5d ; %5d\r\nGyro: %5d ; %5d ; %5d\r\nMagn: %5d ; %5d ; %5d\r\n\r\n",
					speed, TempData, AccData[0], AccData[1], AccData[2],
					GyroData[0], GyroData[1], GyroData[2], MagData[0],
					MagData[1], MagData[2]);

			osSemaphoreRelease(BinarySemHandle);
		}

		// Sending UART log info
		HAL_UART_Transmit(&huart5, str, strlen(str), HAL_MAX_DELAY);

		osDelay(100);
	}
  /* USER CODE END RunTaskDiagnostics */
}
