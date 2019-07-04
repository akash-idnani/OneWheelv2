#include "pid.h"
#include "motiondriver_defs.h"

unsigned long last_time;
float setpoint = 0;
float i_term, last_err, last_input;
float kP, kI, kD;
float out_min, out_max;

int sample_time = 20;
float GAIN_FACTOR = 0.05f;

float pid_compute(float input) {
    float error = setpoint - input;
    i_term += kI * error;
    if (i_term > out_max) i_term = out_max;
    else if (i_term < out_min) i_term = out_min;

    double deriv_input = input - last_input;

    float output = kP * error + i_term - kD * deriv_input;
    if (output > out_max) output = out_max;
    else if (output < out_min) output = out_min;

    last_err = error;
    last_input = input;

    return output;
}

void pid_set_tunings(float new_kp, float new_ki, float new_kd) {
    float sample_time_in_sec = ((float) sample_time) / 1000;

    kP = new_kp * GAIN_FACTOR;
    kI = new_ki * sample_time_in_sec * GAIN_FACTOR;
    kD = new_kd / sample_time_in_sec * GAIN_FACTOR;
}

void set_sample_time(int new_sample_time) {
    float ratio  = (float) new_sample_time / (float) sample_time;
    kI *= ratio;
    kD /= ratio;
    sample_time = (unsigned long) new_sample_time;
}

void pid_set_output_limits(float min, float max) {
    out_min = min;
    out_max = max;

    if (i_term > out_max) i_term = out_max;
    else if (i_term < out_min) i_term = out_min;
}