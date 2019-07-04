#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include <inttypes.h>

#include <esp_timer.h>
#include <driver/dac.h>

#include "motiondriver_defs.h"
#include "mpu_reader.h"
#include "motor_controller.h"
#include "bluetooth.h"
#include "nvs_handler.h"
#include "pid.h"

#define PIN_REVERSE 18

static int is_rolling = 0;

void delay_ms(unsigned long num_ms) {
    vTaskDelay(num_ms / portTICK_RATE_MS);
}

void get_ms(unsigned long *count) {
    *count = esp_timer_get_time() / 1000;
}

void on_new_pid_gains(uint16_t prop, uint16_t integral, uint16_t deriv) {
    printf("%" PRIu16 ", ", prop);
    printf("%" PRIu16 ", ", integral);
    printf("%" PRIu16 "\n", deriv);

    set_nvs(prop, integral, deriv);
    pid_set_tunings(prop, integral, deriv);
}

void get_pid_gains(uint16_t* prop, uint16_t* integral, uint16_t* deriv) {
    read_nvs(prop, integral, deriv);
}

int euler_count = 0;
void on_new_euler(long* euler) {
    float new_motor_out = pid_compute(euler[1] / 65536);
    set_motor(new_motor_out);

    if (euler_count++ > 5) {
        send_euler(euler);
        send_info(new_motor_out < 0 ? -new_motor_out : new_motor_out, new_motor_out < 0, 
                50, 10);
        euler_count = 0;
    }
}

void app_main() {
    init_ble();
    //reset_nvs();

    uint16_t kP, kI, kD;
    read_nvs(&kP, &kI, &kD);

    pid_set_tunings(kP, kI, kD);
    pid_set_output_limits(-255, 255);

    motor_controller_init();
    
    //TaskHandle_t pid_task_handle = NULL;
    //xTaskCreate(euler_reader, "PID", 2048, on_new_euler, 6, &pid_task_handle); 
    
    TaskHandle_t speed_reader_task_handle = NULL;
    xTaskCreate(speed_reader, "Speed", 2048, &is_rolling, 5, &speed_reader_task_handle); 

}