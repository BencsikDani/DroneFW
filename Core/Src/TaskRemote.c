#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "Globals.h"

extern osSemaphoreId BinarySemHandle;
extern osSemaphoreId RemoteSemHandle;
extern UART_HandleTypeDef huart2;

void TaskRemote(void const *argument)
{
	static uint16_t rcValue[IBUS_MAXCHANNELS];// Output values of the channels (1000 ... 2000)
	static bool localProcessRemoteBuffer;

	HAL_UART_Receive_IT(&huart2, &RemoteBuffer, 1);

	/* Infinite loop */
	while (1)
	{
		if (osSemaphoreWait(RemoteSemHandle, osWaitForever) == osOK)
		{
			if (ProcessRemoteBuffer)
			{
				localProcessRemoteBuffer = true;
				ProcessRemoteBuffer = false;
			}

			osSemaphoreRelease(RemoteSemHandle);
		}

		if (localProcessRemoteBuffer)
		{
			// And cycle through the raw data and convert it to actual integer values
			// ibus pattern example:
			// i=0  1     2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21  22 23 24  25  26 27  28 28  30 31
			//   20 40    DB 5  DC 5  54 5  DC 5  E8 3  D0 7  D2 5  E8 3  DC 5  DC 5   DC 5   DC 5   DC 5   DC 5   DA F3
			// | Header | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 | CH9 | CH10 | CH11 | CH12 | CH13 | CH14 | Checksum |
			for (int i = 0; i < IBUS_MAXCHANNELS; i++)
				rcValue[i] = (ibusData[3 + 2 * i] << 8) + ibusData[2 + 2 * i];

			// Setting the speed
			if (osSemaphoreWait(BinarySemHandle, osWaitForever) == osOK)
			{
				if (AccData[2] > 160)
					speed = rcValue[2] / 20;
				else
					speed = 50;

				osSemaphoreRelease(BinarySemHandle);
			}

			// Setting PWM speed
			TIM3->CCR1 = (uint32_t) speed;

			localProcessRemoteBuffer = false;

			osDelay(10);
		}
	}
}
