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
#include "bluetooth.h"
#include "nvs_handler.h"
#include "pid.h"

#define PIN_REVERSE 18


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
}

void get_pid_gains(uint16_t* prop, uint16_t* integral, uint16_t* deriv) {
    read_nvs(prop, integral, deriv);
}

void set_motor(float value) {
    if (value < 0) {
        value = -value;
        gpio_set_level(PIN_REVERSE, 1);
    } else {
        gpio_set_level(PIN_REVERSE, 0);
    }

    dac_output_voltage(DAC_CHANNEL_2, 255 - value);
}

int euler_count = 0;
void on_new_euler(long* euler) {
    float new_motor_out = pid_compute(euler[1] / 65536);
    set_motor(new_motor_out);
    // printf("%.1f \n", new_motor_out);

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

    pid_set_tunings(kP, kI, kD);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_REVERSE);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PIN_REVERSE, 0);

    dac_output_enable(DAC_CHANNEL_2);

    TaskHandle_t pid_task_handle = NULL;
    xTaskCreate(euler_reader, "PID", 2048, on_new_euler, 5, &pid_task_handle); 

    
    // uint8_t output_data=0;
    // int     read_raw;
    // esp_err_t r;

    // gpio_num_t adc_gpio_num;

    // r = adc2_pad_get_io_num( 7, &adc_gpio_num );
    // assert( r == ESP_OK );

    // //be sure to do the init before using adc2. 
    // printf("adc2_init...\n");
    // adc2_config_channel_atten( 7, ADC_ATTEN_0db );  

    // dac_output_enable(DAC_CHANNEL_2);

    // dac_out_voltage(DAC_CHANNEL_2, 255);
    // int count = 255;
    // delay_ms(100);
    // while (count > 200) {
    //     dac_out_voltage(DAC_CHANNEL_2, count--);
    //     if (count < 0) count = 0;
    //     printf("%d\n", count);

    //     r = adc2_get_raw( 7, ADC_WIDTH_12Bit, &read_raw);
    //     if ( r == ESP_OK ) {
    //         printf("%d: %d\n", output_data, read_raw );
    //     } else if ( r == ESP_ERR_INVALID_STATE ) {
    //         printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
    //     } else if ( r == ESP_ERR_TIMEOUT ) {
    //         //This can not happen in this example. But if WiFi is in use, such error code could be returned.
    //         printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
    //     } else {
    //         printf("%s\n", esp_err_to_name(r));
    //     }

    //     delay_ms(100);
    // }

}