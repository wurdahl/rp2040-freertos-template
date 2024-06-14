#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
//#include "error.h"

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define I2C_PORT i2c0
#define SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define SCL_PIN PICO_DEFAULT_I2C_SCL_PIN

static SemaphoreHandle_t i2cMutex = NULL;
bool initalized = false;

bool i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, uint8_t device_addr){

    xSemaphoreTake( i2cMutex, portMAX_DELAY );
    //{

    // Write the register address to the device
    int write_result = i2c_write_blocking(I2C_PORT, device_addr, &reg_addr, 1, true);
    if (write_result == PICO_ERROR_GENERIC || write_result != 1) {
        return false;
    }

    // Read the data from the device
    int read_result = i2c_read_blocking(I2C_PORT, device_addr, reg_data, len, true);
    if (read_result == PICO_ERROR_GENERIC || read_result != len) {
        return false;
    }
    //}
    xSemaphoreGive( i2cMutex );

    return true;

}

bool i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, uint8_t device_addr){

    xSemaphoreTake( i2cMutex, portMAX_DELAY );
    
    // Allocate a buffer to hold the register address and the data
    uint8_t *buffer = malloc(len + 1);
    if (buffer == NULL) {
        printf("Failed to allocate memory for I2C buffer\n");
        return false;
    }

    // Set the first byte of the buffer to the register address
    buffer[0] = reg_addr;
    
    // Copy the data to the buffer
    for (size_t i = 0; i < len; i++) {
        buffer[i + 1] = reg_data[i];
    }

    int result = i2c_write_blocking(I2C_PORT, device_addr, buffer, len + 1, false);

    free(buffer);

    // Check the result of the write operation
    if (result == PICO_ERROR_GENERIC || result != (len + 1)) {
        return false;
    }
    
    xSemaphoreGive( i2cMutex );

    return true;

}

bool init_i2cLib(){
    if(i2cMutex == NULL){
        i2cMutex = xSemaphoreCreateMutex();
    }

    if(initalized){
        return true;
    }else initalized = true;

    xSemaphoreTake( i2cMutex, portMAX_DELAY );
    // Initialize I2C at 100 kHz
    // beware 100 kHZ may more stable
    // error with ACE 3.0 may cause the actual frequency to be much lower
    i2c_init(I2C_PORT, 100 * 1000);

    // Set up I2C pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    xSemaphoreGive( i2cMutex );

    return true;

}