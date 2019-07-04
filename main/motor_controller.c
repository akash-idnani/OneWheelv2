#include "esp_system.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>

#include <string.h>

#include "motor_controller.h"
#include "motiondriver_defs.h"

void motor_controller_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 
        16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));
}

void speed_reader(void* pvParameters) {
    while(1) {
        delay_ms(500);
        char* test_str = "This is a test string.\n";
        printf(test_str);
        uart_write_bytes(UART_NUM_2, (const char*)test_str, strlen(test_str));
    }
}

void set_brake(uint32_t strength) {
}

void set_motor(float value) {
}