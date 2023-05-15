#include "cmsis_os.h"
#include "Globals.h"
#include "IMU/MPU9250.h"
#include "IMU/BMP280.h"

#include "Debug.h"

extern SPI_HandleTypeDef hspi2;
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
			//MPU9250_GetData(AccData, &TempData, GyroData, MagData, false);
			MPU_readRawData(&hspi2, &MPU9250);
			MPU_readProcessedData(&hspi2, &MPU9250);

			AccData[0] = MPU9250.sensorData.ax;
			AccData[1] = MPU9250.sensorData.ay;
			AccData[2] = MPU9250.sensorData.az;
			TempData = MPU9250.sensorData.temp;
			GyroData[0] = MPU9250.sensorData.gx;
			GyroData[1] = MPU9250.sensorData.gy;
			GyroData[2] = MPU9250.sensorData.gz;


			BMP280_measure(&BMP280);

			BMP_Temp = BMP280.measurement.temperature;
			BMP_Pres = BMP280.measurement.pressure;
			BMP_Alt = BMP280.measurement.altitude;

			//Log("SenDat - IMutRelease");
			//osMutexRelease(ImuMutexHandle);
			//Log("SenDat - IMutReleased");
		}
		osMutexRelease(ImuMutexHandle);

		osDelay(100);
	}
}
