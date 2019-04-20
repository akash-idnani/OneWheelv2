#include "pid.h"
#include "motiondriver_defs.h"

unsigned long last_time;
float setpoint = 0;
float err_sum, last_err;
float kP, kI, kD;

float pid_compute(float input) {
    unsigned long now;
    get_ms(&now);
    float time_change = (float) now - last_time;

    float error = setpoint - input;
    err_sum += (error * time_change);
    double deriv_err = (error - last_err) / time_change;

    float output = kP * error + kI * err_sum + kD * deriv_err;

    last_err = error;
    last_time = now;

    if (output > 255) return 255;
    if (output < -255) return -255;
    return output;
}

void pid_set_tunings(float new_kp, float new_ki, float new_kd) {
    kP = new_kp;
    kI = new_ki;
    kD = new_kd;
}