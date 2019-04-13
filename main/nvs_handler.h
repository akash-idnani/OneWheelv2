#ifndef NVS_HANDLER_H
#define NVS_HANDLER_H

#include <inttypes.h>

void reset_nvs();

void read_nvs(uint16_t* prop, uint16_t* integral, uint16_t* deriv);
void set_nvs(uint16_t prop, uint16_t integral, uint16_t deriv);

#endif