#include "mpu6050.h"
#include "math.h"


extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;


uint8_t MPU_init(MPU6050_data *sensor)
{
	uint8_t a;
	uint8_t data = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &a, 1, 100);
	sensor->DataReady = 0;

	if (a == 0x68 || a == 0x70){
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);
		return 1;
		}
	else
		return 0;


}

void DMA_calis_i2c(MPU6050_data *sensor)
{
    sensor->DataReady = 0;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, 0x3B, 1, (uint8_t*)sensor->RxBuffer, 14);

    if(status == HAL_BUSY) {
        __NOP();
    }
}

void MPU_data_isle(MPU6050_data *sensor){

	int16_t Accel_X_RAW = (int16_t)(sensor->RxBuffer[0] << 8 | sensor->RxBuffer[1]);
	int16_t Accel_Y_RAW = (int16_t)(sensor->RxBuffer[2] << 8 | sensor->RxBuffer[3]);
	int16_t Accel_Z_RAW = (int16_t)(sensor->RxBuffer[4] << 8 | sensor->RxBuffer[5]);

	int16_t Gyro_X_RAW = (int16_t)(sensor->RxBuffer[8] << 8 | sensor->RxBuffer[9]);
	int16_t Gyro_Y_RAW = (int16_t)(sensor->RxBuffer[10] << 8 | sensor->RxBuffer[11]);
	int16_t Gyro_Z_RAW = (int16_t)(sensor->RxBuffer[12] << 8 | sensor->RxBuffer[13]);

	sensor->Accel_X_g = Accel_X_RAW / 16384.0f;
	sensor->Accel_Y_g = Accel_Y_RAW / 16384.0f;
	sensor->Accel_Z_g = Accel_Z_RAW / 16384.0f;
	sensor->Gyro_X_deg = Gyro_X_RAW / 131.0f;
	sensor->Gyro_Y_deg = Gyro_Y_RAW / 131.0f;
	sensor->Gyro_Z_deg = Gyro_Z_RAW / 131.0f;

}

void MPU_ivme_Filte_Init(LowPass_data *lp){
	lp->alpha = 0.1f;//bunu deneyerek degistir
	lp->ilk_kontrol = 0;

}

void MPU_ivme_Filte(MPU6050_data *sensor, LowPass_data *lp){
	if(!(lp->ilk_kontrol)){
		lp->oncekiX = sensor->Accel_X_g;
		lp->oncekiY = sensor->Accel_Y_g;
		lp->oncekiZ = sensor->Accel_Z_g;
		lp->ilk_kontrol = 1;
	}
	sensor->Accel_X_g_filtreli = (sensor->Accel_X_g * lp->alpha) + ((1-lp->alpha) * lp->oncekiX);
	sensor->Accel_Y_g_filtreli = (sensor->Accel_Y_g * lp->alpha) + ((1-lp->alpha) * lp->oncekiY);
	sensor->Accel_Z_g_filtreli = (sensor->Accel_Z_g * lp->alpha) + ((1-lp->alpha) * lp->oncekiZ);

	lp->oncekiX = sensor->Accel_X_g_filtreli;
	lp->oncekiY = sensor->Accel_Y_g_filtreli;
	lp->oncekiZ = sensor->Accel_Z_g_filtreli;
}

void MPU_Gyro_Filte_Init(HighPass_data *hp){
	hp->beta = 0.9f;//bunu deneyerek degistir
	hp->ilk_kontrol = 0;

}

void MPU_Gyro_Filte(MPU6050_data *sensor, HighPass_data *hp)
{
    if (!(hp->ilk_kontrol))
    {
        hp->oncekiX_gyro_h = sensor->Gyro_X_deg;
        hp->oncekiY_gyro_h = sensor->Gyro_Y_deg;
        hp->oncekiZ_gyro_h = sensor->Gyro_Z_deg;

        hp->oncekiX_gyro_f = 0.0f;
        hp->oncekiY_gyro_f = 0.0f;
        hp->oncekiZ_gyro_f = 0.0f;

        hp->ilk_kontrol = 1;
        return;
    }

    sensor->Gyro_X_deg_filtreli = (hp->oncekiX_gyro_f * hp->beta) + (hp->beta * (sensor->Gyro_X_deg - hp->oncekiX_gyro_h));
    sensor->Gyro_Y_deg_filtreli = (hp->oncekiY_gyro_f * hp->beta) + (hp->beta * (sensor->Gyro_Y_deg - hp->oncekiY_gyro_h));
    sensor->Gyro_Z_deg_filtreli = (hp->oncekiZ_gyro_f * hp->beta) + (hp->beta * (sensor->Gyro_Z_deg - hp->oncekiZ_gyro_h));

    hp->oncekiX_gyro_f = sensor->Gyro_X_deg_filtreli;
    hp->oncekiY_gyro_f = sensor->Gyro_Y_deg_filtreli;
    hp->oncekiZ_gyro_f = sensor->Gyro_Z_deg_filtreli;

    hp->oncekiX_gyro_h = sensor->Gyro_X_deg;
    hp->oncekiY_gyro_h = sensor->Gyro_Y_deg;
    hp->oncekiZ_gyro_h = sensor->Gyro_Z_deg;
}

void Kalman_MPU_Values(Kalman_data *Kalman) {
    Kalman->w = 0.001f;
    Kalman->bias = 0.003f;
    Kalman->R_measure = 0.03f;
    Kalman->last_angle = 0.0f;
    Kalman->last_bias = 0.0f;
    Kalman->P[0][0] = 0.0f;
    Kalman->P[0][1] = 0.0f;
    Kalman->P[1][0] = 0.0f;
    Kalman->P[1][1] = 0.0f;
}

float MPU_kalman(Kalman_data *Kalman, float ham_angle, float ham_rate, float dt)
{
	Kalman->last_angle += (ham_rate - Kalman->bias) * dt;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->w);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->bias * dt;

	float S = Kalman->P[0][0] + Kalman->R_measure;
	float K[2];
	K[0] = Kalman->P[0][0]/S;
	K[1] = Kalman->P[1][0]/S;

	float diff = ham_angle - Kalman->last_angle;
	Kalman->last_angle += diff * K[0];
	Kalman->last_bias += diff * K[1];

	float P00_temp = Kalman->P[0][0];
	float P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->last_angle;

}

void MPU_Total_Angle(MPU6050_data *myMPU){
	myMPU->total_angle= sqrtf(powf(myMPU->Pitch,2)+ powf(myMPU->Roll,2));
	//formulu kontrol et degisebilir
}

void Lora_Gonder(UART_HandleTypeDef *huart, Telemetri_data *telem, MPU6050_data *sensor)
{
    telem->paket_no++;
    telem->roket_pitch = sensor->Pitch;
    telem->roket_roll = sensor->Roll;
    telem->roket_totalAngle = sensor->total_angle;
    HAL_UART_Transmit_DMA(huart, (uint8_t*)telem, sizeof(Telemetri_data));
}
