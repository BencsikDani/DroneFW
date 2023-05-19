#include "main.h"
#include "cmsis_os.h"
#include "Globals.h"

#include "Debug.h"

extern TIM_HandleTypeDef htim3;
extern osMutexId RemoteDataMutexHandle;

void TaskMotor(void const *argument)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	/* Infinite loop */
	while (1)
	{
		//Log("Mot - RDMutEnter");
		if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			//Log("Mot - RDMutEntered");
			// Setting PWM speed
			TIM3->CCR1 = (uint32_t) 50;
			TIM3->CCR2 = (uint32_t) 50;
			TIM3->CCR4 = (uint32_t) 50;
			TIM3->CCR3 = (uint32_t) (Throttle);

			//Log("Mot - RDMutRelease");
			//osMutexRelease(RemoteDataMutexHandle);
			//Log("Mot - RDMutReleased");
		}
		osMutexRelease(RemoteDataMutexHandle);

		osDelay(100);
	}
}
