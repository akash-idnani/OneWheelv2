#ifndef ONEWHEEL_MOTOR_CONTROLLER_H
#define ONEWHEEL_MOTOR_CONTROLLER_H

#include <driver/dac.h>

#define PIN_REVERSE 18
#define PIN_BRAKE 16
#define DAC_OUT DAC_CHANNEL_2

#define PIN_HALL_YELLOW 23
#define PIN_HALL_GREEN 34
#define PIN_HALL_BLUE 17

void motor_controller_init();
void set_brake(uint32_t strength);

#endif