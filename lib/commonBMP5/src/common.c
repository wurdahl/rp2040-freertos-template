/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

//#include "coines.h" -- this is a proprietary thing for a bosch specific product 
#include "common.h"
#include "bmp5_defs.h"

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"


#define I2C_PORT i2c0
#define SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define SCL_PIN PICO_DEFAULT_I2C_SCL_PIN

/******************************************************************************/
/*!                         Macro definitions                                 */

/*! BMP5 shuttle id */
#define BMP5_SHUTTLE_ID_PRIM  UINT16_C(0x1B3)
#define BMP5_SHUTTLE_ID_SEC   UINT16_C(0x1D3)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    //uint8_t device_addr = BMP5_I2C_ADDR_PRIM;

   // Write the register address to the device
    int write_result = i2c_write_blocking(I2C_PORT, device_addr, &reg_addr, 1, TRUE);
    if (write_result == PICO_ERROR_GENERIC || write_result != 1) {
        return BMP5_E_COM_FAIL;
    }

    // Read the data from the device
    int read_result = i2c_read_blocking(I2C_PORT, device_addr, reg_data, length, FALSE);
    if (read_result == PICO_ERROR_GENERIC || read_result != length) {
        return BMP5_E_COM_FAIL;
    }

    return BMP5_OK;
}

/*!
 * I2C write function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    //uint8_t device_addr = BMP5_I2C_ADDR_PRIM;


    (void)intf_ptr;

    // Allocate a buffer to hold the register address and the data
    uint8_t *buffer = malloc(length + 1);
    if (buffer == NULL) {
        printf("Failed to allocate memory for I2C buffer\n");
        return BMP5_E_COM_FAIL;
    }

    // Set the first byte of the buffer to the register address
    buffer[0] = reg_addr;
    
    // Copy the data to the buffer
    for (size_t i = 0; i < length; i++) {
        buffer[i + 1] = reg_data[i];
    }

    int result = i2c_write_blocking(I2C_PORT, device_addr, buffer, length + 1, false);

    free(buffer);

    // Check the result of the write operation
    if (result == PICO_ERROR_GENERIC || result != (length + 1)) {
        return BMP5_E_COM_FAIL;
    }

    return BMP5_OK;
}

/*!
 * SPI read function map to COINES platform
 */
// BMP5_INTF_RET_TYPE bmp5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t*)intf_ptr;

//     (void)intf_ptr;

//     return coines_read_spi(COINES_SPI_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)length);
// }

/*!
 * SPI write function map to COINES platform
 */
// BMP5_INTF_RET_TYPE bmp5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t*)intf_ptr;

//     (void)intf_ptr;

//     return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
// }

/*!
 * Delay function map to COINES platform
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    busy_wait_us_32(period); //CHANGED!! - PERHAPS MAKE THIS NOT A BUST WAIT
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP5_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP5_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMP5_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_CHIP_ID)
        {
            printf("Error [%d] : Invalid chip id\r\n", rslt);
        }
        else if (rslt == BMP5_E_POWER_UP)
        {
            printf("Error [%d] : Power up error\r\n", rslt);
        }
        else if (rslt == BMP5_E_POR_SOFTRESET)
        {
            printf("Error [%d] : Power-on reset/softreset failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_POWERMODE)
        {
            printf("Error [%d] : Invalid powermode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp5_interface_init(struct bmp5_dev *bmp5_dev, uint8_t intf)
{
    printf("SDA_PIN: %d, SCL_PIN: %d\n", SDA_PIN, SCL_PIN);

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

    dev_addr = BMP5_I2C_ADDR_PRIM;
    bmp5_dev->read = bmp5_i2c_read;
    bmp5_dev->write = bmp5_i2c_write;
    bmp5_dev->intf = BMP5_I2C_INTF;

    /* Holds the I2C device addr or SPI chip selection */
    bmp5_dev->intf_ptr = &dev_addr;

    /* Configure delay in microseconds */
    bmp5_dev->delay_us = bmp5_delay_us;

    return BMP5_OK;
    // int8_t rslt = BMP5_OK;
    // int16_t result;
    // struct coines_board_info board_info;

    // if (bmp5_dev != NULL)
    // {
    //     result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);

    //     if (result < COINES_SUCCESS)
    //     {
    //         printf(
    //             "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
    //             " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
    //         exit(result);
    //     }

    //     result = coines_get_board_info(&board_info);

    //     if (result == COINES_SUCCESS)
    //     {
    //         if ((board_info.shuttle_id != BMP5_SHUTTLE_ID_PRIM) && (board_info.shuttle_id != BMP5_SHUTTLE_ID_SEC))
    //         {
    //             printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
    //             printf("\nShuttle ID : 0x%x\n", board_info.shuttle_id);
    //         }
    //     }

    //     (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
    //     coines_delay_msec(100);

    //     /* Bus configuration : I2C */
    //     if (intf == BMP5_I2C_INTF)
    //     {
    //         printf("I2C Interface\n");

    //         dev_addr = BMP5_I2C_ADDR_PRIM;
    //         bmp5_dev->read = bmp5_i2c_read;
    //         bmp5_dev->write = bmp5_i2c_write;
    //         bmp5_dev->intf = BMP5_I2C_INTF;

    //         /* SDO pin is made low */
    //         (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

    //         (void)coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    //         (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
    //     }
    //     /* Bus configuration : SPI */
    //     else if (intf == BMP5_SPI_INTF)
    //     {
    //         printf("SPI Interface\n");

    //         dev_addr = COINES_SHUTTLE_PIN_7;
    //         bmp5_dev->read = bmp5_spi_read;
    //         bmp5_dev->write = bmp5_spi_write;
    //         bmp5_dev->intf = BMP5_SPI_INTF;
    //         (void)coines_set_pin_config(COINES_SHUTTLE_PIN_7, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    //         (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
    //     }

    //     coines_delay_msec(100);

    //     (void)coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

    //     coines_delay_msec(100);

    //     /* Holds the I2C device addr or SPI chip selection */
    //     bmp5_dev->intf_ptr = &dev_addr;

    //     /* Configure delay in microseconds */
    //     bmp5_dev->delay_us = bmp5_delay_us;
    // }
    // else
    // {
    //     rslt = BMP5_E_NULL_PTR;
    // }

    //return rslt;
}

// void bmp5_coines_deinit(void)
// {
//     (void)fflush(stdout);

//     (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
//     coines_delay_msec(100);

//     /* Coines interface reset */
//     coines_soft_reset();
//     coines_delay_msec(100);

//     (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
// }
