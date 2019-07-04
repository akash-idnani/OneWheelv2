#ifndef PID_H
#define PID_h

float pid_compute(float input);

void pid_set_tunings(float kP, float kI, float kD);
void pid_set_output_limits(float min, float max);

#endif
