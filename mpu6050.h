#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR (0x68 << 1)
#define Lora_AUX_Port GPIOA
#define Lora_AUX_Pin GPIO_PIN_4


typedef struct {
	volatile uint8_t RxBuffer[14];

    volatile uint8_t DataReady;

    float Accel_X_g;
    float Accel_Y_g;
    float Accel_Z_g;

    float Accel_X_g_filtreli;
    float Accel_Y_g_filtreli;
    float Accel_Z_g_filtreli;

    float Gyro_X_deg;
    float Gyro_Y_deg;
    float Gyro_Z_deg;

    float Gyro_X_deg_filtreli;
    float Gyro_Y_deg_filtreli;
    float Gyro_Z_deg_filtreli;

    float Roll;
    float Pitch;

    float total_angle;
} MPU6050_data;


typedef struct{
	float w, bias, R_measure, last_angle, last_bias, P[2][2];

}Kalman_data;

typedef struct{
	float alpha;
	float oncekiX, oncekiY, oncekiZ;
	uint8_t ilk_kontrol;

}LowPass_data;

typedef struct{
	float beta;
	float oncekiX_gyro_h, oncekiY_gyro_h, oncekiZ_gyro_h;//h = ham onceki veriler
	float oncekiX_gyro_f, oncekiY_gyro_f, oncekiZ_gyro_f;//f = filtrelenmis onceki veriler
	uint8_t ilk_kontrol;

}HighPass_data;

typedef struct{
	//buraya gelecek verileri ekle
	uint16_t paket_no;
	float roket_roll;
	float roket_pitch;
	float roket_totalAngle;
}Telemetri_data;


uint8_t MPU_init(MPU6050_data *sensor);

void DMA_calis_i2c(MPU6050_data *sensor);

void MPU_data_isle(MPU6050_data *sensor);

void MPU_ivme_Filte_Init(LowPass_data *lp);

void MPU_ivme_Filte(MPU6050_data *sensor, LowPass_data *lp);

void MPU_Gyro_Filte_Init(HighPass_data *hp);

void MPU_Gyro_Filte(MPU6050_data *sensor, HighPass_data *hp);

void Kalman_MPU_Values(Kalman_data *Kalman);

float MPU_kalman(Kalman_data *Kalman, float ham_angle, float ham_rate, float dt);

void MPU_Total_Angle(MPU6050_data *myMPU);

void Lora_Gonder(UART_HandleTypeDef *huart, Telemetri_data *telem, MPU6050_data *sensor);


#endif
