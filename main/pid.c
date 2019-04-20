#include "pid.h"
#include "motiondriver_defs.h"

unsigned long last_time;
float setpoint = 0;
float err_sum, last_err;
float kP, kI, kD;

int sample_time = 5;

float pid_compute(float input) {
    float error = setpoint - input;
    err_sum += error;
    double deriv_err = error - last_err;

    float output = kP * error + kI * err_sum + kD * deriv_err;

    last_err = error;

    if (output > 255) return 255;
    if (output < -255) return -255;
    return output;
}

void pid_set_tunings(float new_kp, float new_ki, float new_kd) {
    float sample_time_in_sec = ((float) sample_time) / 1000;

    kP = new_kp;
    kI = new_ki * sample_time_in_sec;
    kD = new_kd / sample_time_in_sec;
}

void set_sample_time(int new_sample_time) {
    float ratio  = (float) new_sample_time / (float) sample_time;
    kI *= ratio;
    kD /= ratio;
    sample_time = (unsigned long) new_sample_time;
}