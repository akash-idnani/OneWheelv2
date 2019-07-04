#include "esp_system.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include <freertos/queue.h>

#include <string.h>

#include "motor_controller.h"
#include <comm_uart.h>
#include <bldc_interface.h>
#include <bldc_interface_uart.h>
#include "motiondriver_defs.h"

void bldc_val_received(mc_values *val) {
    
}

void motor_controller_init() {
    comm_uart_init();
    bldc_interface_set_rx_value_func(bldc_val_received);
}

void speed_reader(void* pvParameters) {
    while (1) {
        for (int i = -10000; i < 10000; i++) {
            delay_ms(2);
            bldc_interface_set_rpm(i);
        }
        for (int i = 10000; i > -10000; i--) {
            delay_ms(2);
            bldc_interface_set_rpm(i);
        }
    }
}

void set_brake(uint32_t strength) {
}

void set_motor(float value) {
}