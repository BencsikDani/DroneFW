#include "main.h"
#include "cmsis_os.h"
#include "Globals.h"

extern osSemaphoreId RemoteDataSemHandle;

void TaskMotor(void const *argument)
{
	/* Infinite loop */
	while (1)
	{
		Log("Mot - RDSemEnter");
		if (osSemaphoreWait(RemoteDataSemHandle, osWaitForever) == osOK)
		{
			//Log("Mot - RDSemEnter");
			// Setting PWM speed
			TIM3->CCR1 = (uint32_t) Thrust;

			Log("Mot - RDSemRelease");
			osSemaphoreRelease(RemoteDataSemHandle);
			//Log("Mot - RDSemReleaseD");
		}

		osDelay(1);
	}
}
