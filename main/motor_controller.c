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
    printf("%f %f\n", val->current_motor, val->duty_now);
}

void motor_controller_init() {
    comm_uart_init();
    bldc_interface_set_rx_value_func(bldc_val_received);
}

void set_brake(uint32_t strength) {
}

int i = 0;
void set_motor(float value) {
    bldc_interface_set_duty_cycle(value);
    if (i++ == 500) {
        bldc_interface_get_values();
        i = 0;
    }
}