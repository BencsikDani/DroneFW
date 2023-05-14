#include "cmsis_os.h"
#include "Globals.h"
#include "MPU9250.h"

extern osSemaphoreId ImuSemHandle;

void TaskSensorData(void const *argument)
{
	/* Infinite loop */
	while (1)
	{
		Log("SenDat - ISemEnter");
		if (osSemaphoreWait(ImuSemHandle, osWaitForever) == osOK)
		{
			//Log("SenDat - ISemEntered");
			MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);

			Log("SenDat - ISemRelease");
			osSemaphoreRelease(ImuSemHandle);
			//Log("SenDat - ISemReleased");
		}

		osDelay(100);
	}
}
