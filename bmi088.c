#include "bmi088.h"
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

// PINS
#define ACCEL_CS_PORT GPIOC
#define ACCEL_CS_PIN  GPIO_PIN_5
#define GYRO_CS_PORT  GPIOC
#define GYRO_CS_PIN   GPIO_PIN_4

// Accelometer registers
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_DATA_START   0x12
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
// Gyro registers
#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_DATA_START  0x02
#define BMI088_GYRO_RANGE       0x0F
#define BMI088_GYRO_BW          0x10


void BMI088_Accel_Write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);
}


void BMI088_Accel_Read(uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx_addr = reg | 0x80;
    uint8_t dummy = 0xFF;

    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 100);
    HAL_SPI_Transmit(&hspi1, &dummy, 1, 100);
    HAL_SPI_Receive(&hspi1, data, len, 100);
    HAL_GPIO_WritePin(ACCEL_CS_PORT, ACCEL_CS_PIN, GPIO_PIN_SET);
}

void BMI088_Gyro_Write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data}; // Clear MSB for Write
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

void BMI088_Gyro_Read(uint8_t reg, uint8_t *data, uint16_t len) {
    uint8_t tx_addr = reg | 0x80; // Set MSB for Read

    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, 100); // Send Address
    HAL_SPI_Receive(&hspi1, data, len, 100);    // Read actual data directly
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}


uint8_t BMI088_Init(void) {
    uint8_t chip_id = 0;

    BMI088_Accel_Read(BMI088_ACC_CHIP_ID, &chip_id, 1);
    if (chip_id != 0x1E) return 1;

    BMI088_Accel_Write(BMI088_ACC_PWR_CONF, 0x00);
    HAL_Delay(50);
    BMI088_Accel_Write(BMI088_ACC_PWR_CTRL, 0x04);
    HAL_Delay(50);

    BMI088_Gyro_Read(BMI088_GYRO_CHIP_ID, &chip_id, 1);
    if (chip_id != 0x0F) return 2;

    return 0;
}

// Public Functions
void BMI088_Read_Accel_G(float *accel_in_mg) {
    uint8_t raw[6];

    BMI088_Accel_Read(BMI088_ACC_DATA_START, raw, 6);

    int16_t Accel_X_int16 = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t Accel_Y_int16 = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t Accel_Z_int16 = (int16_t)((raw[5] << 8) | raw[4]);

    accel_in_mg[0] = ((float)Accel_X_int16 / 32768.0f) * 24.0f;
    accel_in_mg[1] = ((float)Accel_Y_int16 / 32768.0f) * 24.0f;
    accel_in_mg[2] = ((float)Accel_Z_int16 / 32768.0f) * 24.0f;
}

void BMI088_Read_Gyro_DPS(float *gyro_dps) {
    uint8_t raw[6];

    BMI088_Gyro_Read(BMI088_GYRO_DATA_START, raw, 6);

    uint16_t rate_X = (uint16_t)((raw[1] << 8) | raw[0]);
    uint16_t rate_Y = (uint16_t)((raw[3] << 8) | raw[2]);
    uint16_t rate_Z = (uint16_t)((raw[5] << 8) | raw[4]);

    gyro_dps[0] = ((float)rate_X / 32768.0f) * 2000.0f;
    gyro_dps[1] = ((float)rate_Y / 32768.0f) * 2000.0f;
    gyro_dps[2] = ((float)rate_Z / 32768.0f) * 2000.0f;

}
