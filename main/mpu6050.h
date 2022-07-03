#ifndef MPU_6050_H
#define MPU_6050_H
#include <math.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// #define MPU6050_SENSOR_ADDR 0x68  /*!< Slave address of the MPU6050 sensor */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define MPU6050_WHO_AM_I_REG_ADDR 0x75   /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define PI 3.14159265358979323846

typedef struct {
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
} MPU6050_data_t;

typedef struct {
    int addr;
} MPU6050_t;

esp_err_t i2c_master_init();
esp_err_t mpu6050_register_read(MPU6050_t mpu, uint8_t reg_addr, uint8_t* data, size_t len);
esp_err_t mpu6050_register_write_byte(MPU6050_t mpu, uint8_t reg_addr, uint8_t data);

void mpu6050_filter(MPU6050_data_t* data, const MPU6050_data_t shift);
void mpu6050_read(MPU6050_t mpu, MPU6050_data_t* data);
void mpu6050_calibrate(MPU6050_t mpu, MPU6050_data_t* err);

#endif