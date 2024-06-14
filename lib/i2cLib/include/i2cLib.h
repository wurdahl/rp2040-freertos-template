#include <stdio.h>

bool i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, uint8_t device_addr);

bool i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, uint8_t device_addr);

bool init_i2cLib();