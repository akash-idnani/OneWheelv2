#ifndef ONEWHEEL_MPU_READER_H
#define ONEWHEEL_MPU_READER_H

#define I2C_BUFFER_SIZE 512
#define I2C_ACK_EN true

#define DEFAULT_MPU_HZ  (50)

void euler_reader();

#endif