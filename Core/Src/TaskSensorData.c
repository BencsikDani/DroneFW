#include "GY-91/BMP280.h"
#include "GY-91/MPU9250.h"
#include "GY-271/HMC5883L.h"
#include "HCSR04/HCSR04.h"
#include "cmsis_os.h"
#include "math.h"
#include "Globals.h"
#include "Debug.h"

extern SPI_HandleTypeDef hspi2;

extern osSemaphoreId DistSemaphoreHandle;
extern osMutexId MagnMutexHandle;
extern osMutexId RemoteDataMutexHandle;
extern osMutexId ImuMutexHandle;
extern osMutexId DistMutexHandle;
extern osMutexId GpsMutexHandle;

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

		if (osMutexWait(MagnMutexHandle, osWaitForever) == osOK)
		{
			struct Vector res = HMC5883L_readRaw();
			MAG_X_RAW = res.XAxis;
			MAG_Y_RAW = res.YAxis;
			MAG_Z_RAW = res.ZAxis;

			magnitude = sqrtf((float)(MAG_X_RAW * MAG_X_RAW)
										+ (float)(MAG_Y_RAW * MAG_Y_RAW)
										+ (float)(MAG_Z_RAW * MAG_Z_RAW));

			MAG_X_NORM = MAG_X_RAW / magnitude;
			MAG_Y_NORM = MAG_Y_RAW / magnitude;
			MAG_Z_NORM = MAG_Z_RAW / magnitude;

			MAG_dir = atan2f(MAG_X_NORM, MAG_Y_NORM)*180.0f/M_PI;

			MAG_dir += declination;

			if (MAG_dir < 0)
				MAG_dir += 360.0f;
			if (MAG_dir > 360.0f)
				MAG_dir -= 360.0f;
		}
		osMutexRelease(MagnMutexHandle);

		if (!HCSR04.Triggered)
		{
			HCSR04_Trigger(&HCSR04);
			HCSR04.Triggered = true;
		}

		else if (HCSR04.Triggered && osSemaphoreWait(DistSemaphoreHandle, osWaitForever) == osOK)
		{
			if (osMutexWait(DistMutexHandle, osWaitForever) == osOK)
			{
				Distance = HCSR04_Read(&HCSR04);
			}
			osMutexRelease(DistMutexHandle);
			HCSR04.Triggered = false;
		}

		osDelay(100);
	}
}
