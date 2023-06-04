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

	uint8_t ESC1_start_signal;
	uint8_t ESC2_start_signal;
	uint8_t ESC3_start_signal;
	uint8_t ESC4_start_signal;

	TIM3->CCR3 = (uint32_t) (50);
	TIM3->CCR4 = (uint32_t) (50);
	TIM3->CCR1 = (uint32_t) (50);
	TIM3->CCR2 = (uint32_t) (50);

	/* Infinite loop */
	while (1)
	{
		//Log("Mot - RDMutEnter");
		if (osMutexWait(RemoteDataMutexHandle, osWaitForever) == osOK)
		{
			if (SWA < 60)
			{
				HAL_GPIO_WritePin(ESC_DOWN_GPIO_Port, ESC_DOWN_Pin, GPIO_PIN_SET);
				//Rotors = true;
			}

			else
			{
				HAL_GPIO_WritePin(ESC_DOWN_GPIO_Port, ESC_DOWN_Pin, GPIO_PIN_RESET);
				//Rotors = false;
			}


			if (SWD < 60)
			{
				ESC1_start_signal = 51;
				ESC2_start_signal = 51;
				ESC3_start_signal = 51;
				ESC4_start_signal = 51;
			}
			else
			{
				ESC1_start_signal = 51;
				ESC2_start_signal = 54;
				ESC3_start_signal = 52;
				ESC4_start_signal = 53;
			}


			// Setting PWM speed
			if (Rotors)
			{
				TIM3->CCR3 = (uint32_t) ((Throttle * (100-(ESC1_start_signal-1)) / 50) + (2*(ESC1_start_signal-1) - 100));
				TIM3->CCR4 = (uint32_t) ((Throttle * (100-(ESC2_start_signal-1)) / 50) + (2*(ESC2_start_signal-1) - 100));
				TIM3->CCR1 = (uint32_t) ((Throttle * (100-(ESC3_start_signal-1)) / 50) + (2*(ESC3_start_signal-1) - 100));
				TIM3->CCR2 = (uint32_t) ((Throttle * (100-(ESC4_start_signal-1)) / 50) + (2*(ESC4_start_signal-1) - 100));
			}
			else
			{
				TIM3->CCR3 = (uint32_t) (50);
				TIM3->CCR4 = (uint32_t) (50);
				TIM3->CCR1 = (uint32_t) (50);
				TIM3->CCR2 = (uint32_t) (50);
			}
		}
		osMutexRelease(RemoteDataMutexHandle);

		osDelay(100);
	}
}
