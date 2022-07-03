#include "mpu6050.h"

esp_err_t mpu6050_register_read(MPU6050_t mpu, uint8_t reg_addr, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, mpu.addr, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

esp_err_t mpu6050_register_write_byte(MPU6050_t mpu, uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, mpu.addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

esp_err_t i2c_master_init(void) {
    int i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,  // select GPIO specific to your project
        .scl_io_num = 22,  // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  // select frequency specific to your project
    };

    if (i2c_param_config(i2c_master_port, &conf) == ESP_OK)
        ESP_LOGI("MPU6050", "param config ok");
    else {
        ESP_LOGE("MPU6050", "param config failed");
        return ESP_FAIL;
    }

    if (i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0) == ESP_OK)
        ESP_LOGI("MPU6050", "driver install ok");
    else {
        ESP_LOGE("MPU6050", "driver install failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

MPU6050_data_t calibration = (MPU6050_data_t){.accel_x = 0, .accel_y = 0, .accel_z = 0, .gyro_x = 0, .gyro_y = 0, .gyro_z = 0};

void mpu6050_filter(MPU6050_data_t* data, const MPU6050_data_t shift) {
    data->accel_x = ((data->accel_x > 2) ? data->accel_x - 4 : data->accel_x) - shift.accel_x;
    data->accel_y = ((data->accel_y > 2) ? data->accel_y - 4 : data->accel_y) - shift.accel_y;
    data->accel_z = ((data->accel_z > 2) ? data->accel_z - 4 : data->accel_z) - shift.accel_z;
    data->gyro_x = ((data->gyro_x > 250) ? data->gyro_x - 500 : data->gyro_x) - shift.gyro_x;
    data->gyro_y = ((data->gyro_y > 250) ? data->gyro_y - 500 : data->gyro_y) - shift.gyro_y;
    data->gyro_z = ((data->gyro_z > 250) ? data->gyro_z - 500 : data->gyro_z) - shift.gyro_z;
}

void mpu6050_read(MPU6050_t mpu, MPU6050_data_t* data) {
    uint8_t reg[12] = {0};
    mpu6050_register_read(mpu, MPU6050_ACCEL_XOUT_H, &reg[0], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_ACCEL_XOUT_L, &reg[1], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_ACCEL_YOUT_H, &reg[2], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_ACCEL_YOUT_L, &reg[3], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_ACCEL_ZOUT_H, &reg[4], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_ACCEL_ZOUT_L, &reg[5], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_GYRO_XOUT_H, &reg[6], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_GYRO_XOUT_L, &reg[7], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_GYRO_YOUT_H, &reg[8], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_GYRO_YOUT_L, &reg[9], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_GYRO_ZOUT_H, &reg[10], sizeof(uint8_t));
    mpu6050_register_read(mpu, MPU6050_GYRO_ZOUT_L, &reg[11], sizeof(uint8_t));

    *data = (MPU6050_data_t){
        .accel_x = (reg[0] << 8 | reg[1]) / 16384.0,
        .accel_y = (reg[2] << 8 | reg[3]) / 16384.0,
        .accel_z = (reg[4] << 8 | reg[5]) / 16384.0,
        .gyro_x = (reg[6] << 8 | reg[7]) / 131.0,
        .gyro_y = (reg[8] << 8 | reg[9]) / 131.0,
        .gyro_z = (reg[10] << 8 | reg[11]) / 131.0};
    mpu6050_filter(data, calibration);
}

void mpu6050_calibrate(MPU6050_t mpu, MPU6050_data_t* err) {
    MPU6050_data_t acc_err = (MPU6050_data_t){.accel_x = 0, .accel_y = 0, .accel_z = 0, .gyro_x = 0, .gyro_y = 0, .gyro_z = 0};
    const int iter = 200;
    for (int i = 0; i < iter; i++) {
        MPU6050_data_t data;
        mpu6050_read(mpu, &data);
        acc_err.accel_x += data.accel_x;
        acc_err.accel_y += data.accel_y;
        acc_err.accel_z += data.accel_z;
        acc_err.gyro_x += data.gyro_x;
        acc_err.gyro_y += data.gyro_y;
        acc_err.gyro_z += data.gyro_z;
    }
    err->accel_x = acc_err.accel_x / iter;
    err->accel_y = acc_err.accel_y / iter;
    err->accel_z = acc_err.accel_z / iter;
    err->gyro_x = acc_err.gyro_x / iter;
    err->gyro_y = acc_err.gyro_y / iter;
    err->gyro_z = acc_err.gyro_z / iter;
}