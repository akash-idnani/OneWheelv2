#ifndef ONEWHEEL_MOTOR_CONTROLLER_H
#define ONEWHEEL_MOTOR_CONTROLLER_H

void motor_controller_init();
void set_brake(uint32_t strength);
void set_motor(float value);
void speed_reader(void* pvParameters);

#endif