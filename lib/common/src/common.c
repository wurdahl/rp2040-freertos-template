/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmp3.h"
//include coines -- this is a proprietary thing for a bosch specific product 
#include "common.h"

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"

/*! BMP3 shuttle board ID */
#define BMP3_SHUTTLE_ID  0xD3

#define I2C_PORT i2c0
#define SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define SCL_PIN PICO_DEFAULT_I2C_SCL_PIN

/* Variable to store the device address */
static uint8_t dev_addr;



/*!
 * I2C read function - re-written by william for RP2040
 * similar to the write, but we must first write to the device which register we want data from (last arguement must be true to "maintain master control of the bus")
 * then read in the data to a buffer with the correct length
 * ensure that these functions are used in this way
 * ensure I am doing the right level of lowness
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){

    uint8_t device_addr = *(uint8_t*)intf_ptr;

   // Write the register address to the device
    int write_result = i2c_write_blocking(I2C_PORT, device_addr, &reg_addr, 1, true);
    if (write_result == PICO_ERROR_GENERIC || write_result != 1) {
        return BMP3_E_COMM_FAIL;
    }

    // Read the data from the device
    int read_result = i2c_read_blocking(I2C_PORT, device_addr, reg_data, len, false);
    if (read_result == PICO_ERROR_GENERIC || read_result != len) {
        return BMP3_E_COMM_FAIL;
    }

    return BMP3_OK;
}

/*!
 * I2C write function - re-written by william for RP2040
 * This needs to just encapsulate writing data to the bmp given a register address, so should use known i2c address
 * when writing, ADDR is known, buf is a uint8_t array containing the register and the value 
 * need to size the buffer array for sending given the provided length of the message
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    // Allocate a buffer to hold the register address and the data
    uint8_t *buffer = malloc(len + 1);
    if (buffer == NULL) {
        printf("Failed to allocate memory for I2C buffer\n");
        return BMP3_E_COMM_FAIL;
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
        return BMP3_E_COMM_FAIL;
    }

    return BMP3_OK;
}


/*!
 * re-written by william for RP2040
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    busy_wait_us_32(period);
}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

//Re-written by william for RP2040
//Not sure what this needs to do...
//Needs to set-up interface so should just set up neccessary i2c things
// "Function to select the interface between SPI and I2C."
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf){

    printf("SDA_PIN: %d, SCL_PIN: %d\n", SDA_PIN, SCL_PIN);

    // Initialize I2C at 100 kHz
    i2c_init(I2C_PORT, 100 * 1000);
    
    // Set up I2C pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

    // Store the interface pointer for I2C
    bmp3->intf_ptr = &intf;
    bmp3->read = bmp3_i2c_read;  // Assign the read function pointer
    bmp3->write = bmp3_i2c_write;  // Assign the write function pointer
    bmp3->delay_us = bmp3_delay_us;  // Assign a delay function pointer (if necessary)
    bmp3->intf = BMP3_I2C_INTF;  // Define the interface type

    return BMP3_OK;

}

// BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
// {
//     int8_t rslt = BMP3_OK;
//     struct coines_board_info board_info;

//     if (bmp3 != NULL)
//     {
//         int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
//         if (result < COINES_SUCCESS)
//         {
//             printf(
//                 "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
//                 " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
//             exit(result);
//         }

//         result = coines_get_board_info(&board_info);

// #if defined(PC)
//         setbuf(stdout, NULL);
// #endif

//         if (result == COINES_SUCCESS)
//         {
//             if ((board_info.shuttle_id != BMP3_SHUTTLE_ID))
//             {
//                 printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
//             }
//         }

//         (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
//         coines_delay_msec(1000);

//         /* Bus configuration : I2C */
//         if (intf == BMP3_I2C_INTF)
//         {
//             printf("I2C Interface\n");
//             dev_addr = BMP3_ADDR_I2C_PRIM;
//             bmp3->read = bmp3_i2c_read;
//             bmp3->write = bmp3_i2c_write;
//             bmp3->intf = BMP3_I2C_INTF;

//             /* SDO pin is made low */
//             (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
//             (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
//         }
//         /* Bus configuration : SPI */
//         else if (intf == BMP3_SPI_INTF)
//         {
//             printf("SPI Interface\n");
//             dev_addr = COINES_SHUTTLE_PIN_7;
//             bmp3->read = bmp3_spi_read;
//             bmp3->write = bmp3_spi_write;
//             bmp3->intf = BMP3_SPI_INTF;
//             (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
//         }

//         coines_delay_msec(1000);

//         (void)coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

//         coines_delay_msec(1000);

//         bmp3->delay_us = bmp3_delay_us;
//         bmp3->intf_ptr = &dev_addr;
//     }
//     else
//     {
//         rslt = BMP3_E_NULL_PTR;
//     }

//     return rslt;
//}

// void bmp3_coines_deinit(void)
// {
//     (void)fflush(stdout);

//     (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
//     coines_delay_msec(1000);

//     /* Coines interface reset */
//     coines_soft_reset();
//     coines_delay_msec(1000);
//     (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
// }
