#ifndef BMI088_H
#define BMI088_H

#include <stm32f4xx_hal.h>

#define ACCEL_CS_PORT GPIOC
#define ACCEL_CS_PIN  GPIO_PIN_5
#define GYRO_CS_PORT  GPIOC
#define GYRO_CS_PIN   GPIO_PIN_4

#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_DATA_START   0x12
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D

#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_DATA_START  0x02
#define BMI088_GYRO_RANGE       0x0F
#define BMI088_GYRO_BW          0x10


uint8_t BMI088_Init(void);


void BMI088_Read_Accel_G(float *accel_in_mg);

void BMI088_Read_Gyro_DPS(float *gyro_dps);

#endif /* BMI088_H */
