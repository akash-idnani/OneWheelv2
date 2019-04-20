#ifndef PID_H
#define PID_h

float pid_compute(float input);

void pid_set_tunings(float kP, float kI, float kD);

#endif
