#include "cmsis_os.h"
#include "Globals.h"
#include "MPU9250.h"

#include "Debug.h"

extern osMutexId ImuMutexHandle;

void TaskSensorData(void const *argument)
{
	/* Infinite loop */
	while (1)
	{
		//Log("SenDat - IMutEnter");
		if (osMutexWait(ImuMutexHandle, osWaitForever) == osOK)
		{
			//Log("SenDat - IMutEntered");
			MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);

			//Log("SenDat - IMutRelease");
			osMutexRelease(ImuMutexHandle);
			//Log("SenDat - IMutReleased");
		}

		osDelay(100);
	}
}
