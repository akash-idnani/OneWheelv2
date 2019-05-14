#include "pid.h"
#include "motiondriver_defs.h"

unsigned long last_time;
float setpoint = 0;
float i_term, last_err, last_input;
float kP, kI, kD;

int sample_time = 5;

float pid_compute(float input) {
    float error = setpoint - input;
    i_term += kI * error;
    double deriv_input = input - last_input;

    float output = kP * error + i_term - kD * deriv_input;

    last_err = error;
    last_input = input;

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