#include "main.h"
#include "cmsis_os.h"
#include "Globals.h"

#include "Debug.h"

extern osMutexId RemoteDataMutexHandle;

void TaskMotor(void const *argument)
{
	/* Infinite loop */
	while (1)
	{
		//Log("Mot - RDMutEnter");
		if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			//Log("Mot - RDMutEntered");
			// Setting PWM speed
			TIM3->CCR1 = (uint32_t) Thrust;

			//Log("Mot - RDMutRelease");
			//osMutexRelease(RemoteDataMutexHandle);
			//Log("Mot - RDMutReleased");
		}
		osMutexRelease(RemoteDataMutexHandle);

		osDelay(100);
	}
}
