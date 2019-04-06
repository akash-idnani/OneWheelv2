#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <inttypes.h>

#include <esp_timer.h>
#include <driver/dac.h>

#include "motiondriver_defs.h"
#include "mpu_reader.h"
#include "bluetooth.h"


void delay_ms(unsigned long num_ms) {
    vTaskDelay(num_ms / portTICK_RATE_MS);
}

void get_ms(unsigned long *count) {
    *count = esp_timer_get_time() / 1000;
}


void reset_nvs() {
    nvs_handle nvs_h;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_h));
    ESP_ERROR_CHECK(nvs_set_u16(nvs_h, "Kp", 10));
    ESP_ERROR_CHECK(nvs_set_u16(nvs_h, "Ki", 11));
    ESP_ERROR_CHECK(nvs_set_u16(nvs_h, "Kd", 12));

    ESP_ERROR_CHECK(nvs_commit(nvs_h));
    nvs_close(nvs_h);
}

void read_nvs(uint16_t* prop, uint16_t* integral, uint16_t* deriv) {
    nvs_handle nvs_h;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &nvs_h));
    ESP_ERROR_CHECK(nvs_get_u16(nvs_h, "Kp", prop));
    ESP_ERROR_CHECK(nvs_get_u16(nvs_h, "Ki", integral));
    ESP_ERROR_CHECK(nvs_get_u16(nvs_h, "Kd", deriv));
    nvs_close(nvs_h);
}

int euler_count = 0;
void on_new_euler(long* euler) {
    if (euler_count++ > 5) {
        // printf("Euler: %ld, %ld, %ld \n", euler[0], euler[1], euler[2]);
        send_euler(euler);
        euler_count = 0;
    }
}

void app_main() {
    init_ble();
    // reset_nvs();

    uint16_t kP, kI, kD;
    read_nvs(&kP, &kI, &kD);

    printf("%" PRIu16 ", ", kP);
    printf("%" PRIu16 ", ", kI);
    printf("%" PRIu16 "\n", kD);

    TaskHandle_t pid_task_handle = NULL;
    xTaskCreate(pid_task, "PID", 2048, on_new_euler, 5, &pid_task_handle);   

    dac_output_enable(DAC_CHANNEL_2);

    int count = 0;
    while (1) {
        count = (count + 1) % 255;
        dac_output_voltage(DAC_CHANNEL_2, 0);
        delay_ms(200);
        printf("%d\n", count);
    }
}