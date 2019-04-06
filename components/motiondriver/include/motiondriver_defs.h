#ifndef MOTIONDRIVER_DEFS_H
#define MOTIONDRIVER_DEFS_H

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, 
        unsigned char length, unsigned char const *data);

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
        unsigned char length, unsigned char *data);

void delay_ms(unsigned long num_ms);
void get_ms(unsigned long *count);

#define log_i printf
#define log_e printf

#define MPU6500
#define EMPL

#endif