#include "cmsis_os.h"
#include "Globals.h"
#include "MPU9250.h"

extern osSemaphoreId BinarySemHandle;

void TaskSensorData(void const *argument)
{
	/* Infinite loop */
	while (1)
	{
		if (osSemaphoreWait(BinarySemHandle, osWaitForever) == osOK)
		{
			MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);

			osSemaphoreRelease(BinarySemHandle);
		}

		osDelay(100);
	}
}
