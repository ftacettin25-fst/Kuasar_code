#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
uint8_t BMI088_Init(void);


void BMI088_Read_Accel_G(float *accel_in_mg);

void BMI088_Read_Gyro_DPS(float *gyro_dps);

#endif /* BMI088_H */
