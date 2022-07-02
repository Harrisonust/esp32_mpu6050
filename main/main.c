#include <math.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#define LED CONFIG_BLINK_LED

static uint8_t led_state = 0;

#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21

#define MPU6050_SENSOR_ADDR 0x68         /*!< Slave address of the MPU6050 sensor */
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

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,  // select GPIO specific to your project
        .scl_io_num = I2C_MASTER_SCL_IO,  // select GPIO specific to your project
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

static void blink(void* par) {
    while (1) {
        // ESP_LOGI("LED", "%s", (led_state) ? "ON" : "OFF");
        gpio_set_level(LED, led_state);
        led_state = !led_state;
        vTaskDelay(400 / portTICK_PERIOD_MS);
    }
}

typedef struct {
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
} MPU6050_data;

MPU6050_data calibration = (MPU6050_data){.accel_x = 0, .accel_y = 0, .accel_z = 0, .gyro_x = 0, .gyro_y = 0, .gyro_z = 0};

void mpu6050_filter(MPU6050_data* data, const MPU6050_data shift) {
    data->accel_x = ((data->accel_x > 2) ? data->accel_x - 4 : data->accel_x) - shift.accel_x;
    data->accel_y = ((data->accel_y > 2) ? data->accel_y - 4 : data->accel_y) - shift.accel_y;
    data->accel_z = ((data->accel_z > 2) ? data->accel_z - 4 : data->accel_z) - shift.accel_z;
    data->gyro_x = ((data->gyro_x > 250) ? data->gyro_x - 500 : data->gyro_x) - shift.gyro_x;
    data->gyro_y = ((data->gyro_y > 250) ? data->gyro_y - 500 : data->gyro_y) - shift.gyro_y;
    data->gyro_z = ((data->gyro_z > 250) ? data->gyro_z - 500 : data->gyro_z) - shift.gyro_z;
}

void mpu6050_read(MPU6050_data* data) {
    uint8_t reg[12] = {0};
    mpu6050_register_read(MPU6050_ACCEL_XOUT_H, &reg[0], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_ACCEL_XOUT_L, &reg[1], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_ACCEL_YOUT_H, &reg[2], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_ACCEL_YOUT_L, &reg[3], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_ACCEL_ZOUT_H, &reg[4], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_ACCEL_ZOUT_L, &reg[5], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_GYRO_XOUT_H, &reg[6], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_GYRO_XOUT_L, &reg[7], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_GYRO_YOUT_H, &reg[8], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_GYRO_YOUT_L, &reg[9], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_GYRO_ZOUT_H, &reg[10], sizeof(uint8_t));
    mpu6050_register_read(MPU6050_GYRO_ZOUT_L, &reg[11], sizeof(uint8_t));

    *data = (MPU6050_data){
        .accel_x = (reg[0] << 8 | reg[1]) / 16384.0,
        .accel_y = (reg[2] << 8 | reg[3]) / 16384.0,
        .accel_z = (reg[4] << 8 | reg[5]) / 16384.0,
        .gyro_x = (reg[6] << 8 | reg[7]) / 131.0,
        .gyro_y = (reg[8] << 8 | reg[9]) / 131.0,
        .gyro_z = (reg[10] << 8 | reg[11]) / 131.0};
    mpu6050_filter(data, calibration);
}

void mpu6050_calibrate(MPU6050_data* err) {
    MPU6050_data acc_err = (MPU6050_data){.accel_x = 0, .accel_y = 0, .accel_z = 0, .gyro_x = 0, .gyro_y = 0, .gyro_z = 0};
    const int iter = 200;
    for (int i = 0; i < iter; i++) {
        MPU6050_data data;
        mpu6050_read(&data);
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

static void mpu6050_thread(void* par) {
    if (i2c_master_init() == ESP_OK)
        ESP_LOGI("MPU6050", "I2C master init success");
    else
        ESP_LOGE("MPU6050", "I2C master init failed");

    mpu6050_calibrate(&calibration);
    ESP_LOGI("MPU6050", "Calibration: %f %f %f %f %f %f", calibration.accel_x, calibration.accel_y, calibration.accel_z, calibration.gyro_x, calibration.gyro_y, calibration.gyro_z);

    double previousTime = 0, currentTime = xTaskGetTickCount();
    double x = 0, y = 0, z = 0, yaw = 0, roll = 0, pitch = 0;
    while (1) {
        MPU6050_data data;
        mpu6050_read(&data);

        previousTime = currentTime;
        currentTime = xTaskGetTickCount();
        double elapsedTime = (currentTime - previousTime) / 1000;

        roll += data.gyro_x * elapsedTime;
        pitch += data.gyro_y * elapsedTime;
        yaw += data.gyro_z * elapsedTime;

        ESP_LOGI("MPU6050", "accel[ %3.2f, %3.2f, %3.2f ]\tgyro[ %3.2f, %3.2f, %3.2f ]\trpy[ %3.2f, %3.2f ,%3.2f ]",
                 data.accel_x, data.accel_y, data.accel_z,
                 data.gyro_x, data.gyro_y, data.gyro_z,
                 roll, pitch, yaw);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
void app_main(void) {
    ESP_LOGI("System", "Example configured to blink GPIO LED!");

    // init
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    xTaskCreate(
        blink,
        "debug_task",
        2048,
        NULL,
        1,
        NULL);

    xTaskCreate(
        mpu6050_thread,
        "mpu6050_task",
        2048,
        NULL,
        1,
        NULL);
}
