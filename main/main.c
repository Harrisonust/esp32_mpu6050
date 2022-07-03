#include <math.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "sdkconfig.h"

#define LED CONFIG_BLINK_LED

static uint8_t led_state = 0;

void blink(void* par) {
    while (1) {
        // ESP_LOGI("LED", "%s", (led_state) ? "ON" : "OFF");
        gpio_set_level(LED, led_state);
        led_state = !led_state;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

extern MPU6050_data_t calibration;
void mpu6050_thread(void* par) {
    if (i2c_master_init() == ESP_OK)
        ESP_LOGI("MPU6050", "I2C master init success");
    else
        ESP_LOGE("MPU6050", "I2C master init failed");

    MPU6050_t mpu1 = (MPU6050_t){.addr = 0x68};

    mpu6050_calibrate(mpu1, &calibration);
    ESP_LOGI("MPU6050", "Calibration: %f %f %f %f %f %f",
             calibration.accel_x, calibration.accel_y, calibration.accel_z,
             calibration.gyro_x, calibration.gyro_y, calibration.gyro_z);
    double previousTime = 0, currentTime = xTaskGetTickCount();
    double yaw = 0, roll = 0, pitch = 0;
    while (1) {
        MPU6050_data_t data;
        mpu6050_read(mpu1, &data);

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
