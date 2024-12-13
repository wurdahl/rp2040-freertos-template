#include <stdio.h>
//#include "pico/stdlib.h"
#include <string.h>
#include <math.h>
#include "bmi08x.h"
#include "commonBMI08X.h"

#include "FreeRTOS.h"
#include "task.h"

#define ACCEL_READING_FREQ      100

/*********************************************************************/
/*                              BMI Macros                           */
/*********************************************************************/

/* Buffer size allocated to store raw FIFO data for accel */
#define BMI08_ACC_FIFO_RAW_DATA_BUFFER_SIZE           UINT16_C(1024)

/* Length of data to be read from FIFO for accel */
#define BMI08_ACC_FIFO_RAW_DATA_USER_LENGTH           UINT16_C(1024)

/* Watermark level for accel */
#define BMI08_ACC_FIFO_WATERMARK_LEVEL                UINT16_C(10)

/* Buffer size allocated to store raw FIFO data */
#define BMI08_FIFO_RAW_DATA_BUFFER_SIZE        UINT16_C(600)

/* Length of data to be read from FIFO */
#define BMI08_FIFO_RAW_DATA_USER_LENGTH        UINT16_C(600)

/* Number of Gyro frames to be extracted from FIFO */
#define BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(100)

/* Number of Accel frames to be extracted from FIFO */

/* (Each frame has 7 bytes: 1 byte header + 6 bytes accel data)
 * Watermark level / Frame size = 500 / 7 = 71
 * Extra frames given to obtain sensortime
 */
#define BMI08_ACC_FIFO_WM_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(75)

/*********************************************************************/
/*                       BMI Global variables                        */
/*********************************************************************/
/*! @brief This structure containing relevant bmi08 info */
struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
struct bmi08_sensor_data bmi08_accel[100] = { { 0 } };

/*! bmi08 accel int config */
struct bmi08_accel_int_channel_cfg accel_int_config;

/*! @brief variable to hold the bmi08 gyro data */
struct bmi08_sensor_data bmi08_gyro[100] = { { 0 } };

/*! bmi08 gyro int config */
struct bmi08_gyro_int_channel_cfg gyro_int_config;


static int8_t init_bmi08(void)
{
    int8_t rslt;

    rslt = bmi08xa_init(&bmi08dev);
    bmi08_error_codes_print_result("bmi08xa_init", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_init", rslt);
    }

    if (rslt == BMI08_OK)
    {
        printf("Uploading config file !\n");
        rslt = bmi08a_load_config_file(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_load_config_file", rslt);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_400_HZ;

        if (bmi08dev.variant == BMI085_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08dev.variant == BMI088_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_12G;
        }

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;

        rslt = bmi08a_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_set_power_mode", rslt);

        rslt = bmi08xa_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08xa_set_meas_conf", rslt);

        bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_125_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_power_mode", rslt);

        rslt = bmi08g_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_meas_conf", rslt);
    }

    return rslt;
}

static int8_t enable_bmi08_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_2;
    accel_int_config.int_type = BMI08_ACCEL_INT_FIFO_WM;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    return rslt;
}

static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    float returnVal = (9.80665f * val * g_range) / half_scale;

    return returnVal;
}

void BMI_ACC_FIFO(void *pvParameters){
    
    //code from main section of accel_fifo_watermark.c example
    int8_t rslt;
    
    uint8_t status = 0;

    /* Variable to set water mark level */
    uint16_t wml = 0;

    /* Initialize FIFO frame structure */
    struct bmi08_fifo_frame fifo_frame = { 0 };

    /* To configure the FIFO accel configurations */
    struct bmi08_accel_fifo_config config;

    /* Number of bytes of FIFO data for accel*/
    uint8_t fifo_data_a[BMI08_ACC_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Number of accelerometer frames */
    uint16_t accel_length = BMI08_ACC_FIFO_WM_EXTRACTED_DATA_FRAME_COUNT;

    /* Variable to index bytes */
    uint16_t idx = 0;

    uint8_t attempt = 1;

    /* Variable to store sensor time value */
    uint32_t sensor_time;

    /* Variable to store available fifo length */
    uint16_t fifo_length;

    /* Gyroscope fifo configurations */
    struct bmi08_gyr_fifo_config gyr_conf = { 0 };

    /* Fifo frame structure */
    struct bmi08_fifo_frame fifo = { 0 };

    /* Number of gyroscope frames */
    uint16_t gyro_length = BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT;

    /* Number of bytes of FIFO data for gyroscope*/
    uint8_t fifo_data_g[BMI08_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Interface given as parameter
     *           For I2C : BMI08_I2C_INTF
     *           For SPI : BMI08_SPI_INTF
     * Sensor variant given as parameter
     *          For BMI085 : BMI085_VARIANT
     *          For BMI088 : BMI088_VARIANT
     */
    rslt = bmi08_interface_init(&bmi08dev, BMI08_I2C_INTF, BMI088_VARIANT);
    bmi08_error_codes_print_result("bmi08_interface_init", rslt);

    

    if (rslt == BMI08_OK)
    {
        rslt = init_bmi08();
        bmi08_error_codes_print_result("init_bmi08", rslt);

        //do a premptive reset of the accel because it might have already been on
        //DOES NOT WORK - WOULD BE REALLY COOL IF IT DID
        // bmi08a_soft_reset(&bmi08dev);
        // rslt = init_bmi08();
        // bmi08_error_codes_print_result("init_bmi08", rslt);

        /*Enable data ready interrupts*/
        rslt = enable_bmi08_interrupt();
        bmi08_error_codes_print_result("enable_bmi08_interrupt", rslt);

        printf("Accel FIFO watermark interrupt data\n");
        if (rslt == BMI08_OK)
        {
            /* Set water mark level */
            rslt = bmi08a_set_fifo_wm(BMI08_ACC_FIFO_WATERMARK_LEVEL, &bmi08dev);
            bmi08_error_codes_print_result("bmi08a_set_fifo_wm", rslt);

            config.accel_en = BMI08_ENABLE;

            /* Set FIFO configuration by enabling accelerometer */
            rslt = bmi08a_set_fifo_config(&config, &bmi08dev);
            bmi08_error_codes_print_result("bmi08a_set_fifo_config", rslt);

            gyr_conf.mode = BMI08_GYRO_FIFO_MODE;
            gyr_conf.tag = BMI08_GYRO_FIFO_TAG_DISABLED;

            rslt = bmi08g_set_fifo_config(&gyr_conf, &bmi08dev);
            bmi08_error_codes_print_result("bmi08g_set_fifo_config", rslt);

            /* Update FIFO structure */
            fifo.data = fifo_data_g;
            fifo.length = BMI08_FIFO_RAW_DATA_USER_LENGTH;

            //Code to run task every .1 seconds
            TickType_t xLastWakeTime;
            const TickType_t xFrequency = ACCEL_READING_FREQ; //every 100 ticks run this thread
            BaseType_t xWasDelayed; //used to notice if the function took too long to execute

            // Initialise the xLastWakeTime variable with the current time.
            xLastWakeTime = xTaskGetTickCount();

            while (true)
            {
                xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

                if(xWasDelayed==pdFALSE){
                    // xSemaphoreTake( printMutex, portMAX_DELAY );
                    // {
                        printf("****ACCELERATION TASK WAS DELAYED****");
                    // }
                    // xSemaphoreGive( printMutex );
                }

                rslt = bmi08a_get_data_int_status(&status, &bmi08dev);
                bmi08_error_codes_print_result("bmi08a_get_data_int_status", rslt);

                if (status & BMI08_ACCEL_FIFO_WM_INT)
                {
                    // xSemaphoreTake( printMutex, portMAX_DELAY );
                    // {
                        printf("\nIteration : %d\n", attempt);

                        /* Update FIFO structure */
                        fifo_frame.data = fifo_data_a;
                        fifo_frame.length = BMI08_ACC_FIFO_RAW_DATA_USER_LENGTH;

                        accel_length = BMI08_ACC_FIFO_WM_EXTRACTED_DATA_FRAME_COUNT;

                        rslt = bmi08a_get_fifo_length(&fifo_length, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08a_get_fifo_length", rslt);

                        rslt = bmi08a_get_fifo_wm(&wml, &bmi08dev);
                        bmi08_error_codes_print_result("bmi08a_get_fifo_length", rslt);

                        printf("Watermark level : %d\n", wml);

                        printf("FIFO buffer size : %d\n", fifo_frame.length);
                        printf("FIFO length available : %d\n", fifo_length);

                        printf("Requested data frames before parsing: %d\n", accel_length);

                        if (rslt == BMI08_OK){
                            /* Read FIFO data */
                            rslt = bmi08a_read_fifo_data(&fifo_frame, &bmi08dev);
                            bmi08_error_codes_print_result("bmi08a_read_fifo_data", rslt);

                            /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                            rslt = bmi08a_extract_accel(bmi08_accel, &accel_length, &fifo_frame, &bmi08dev);
                            bmi08_error_codes_print_result("bmi08a_extract_accel", rslt);

                            printf("Parsed accelerometer frames: %d\n", accel_length);

                            printf("\nFrame_Count, X, Y, Z\n");

                            /* Print the parsed accelerometer data from the FIFO buffer */
                            for (idx = 0; idx < accel_length; idx++)
                            {
                                //printf("%d, %f, %f, %f\n", idx, lsb_to_mps2(bmi08_accel[idx].x,12,16), lsb_to_mps2(bmi08_accel[idx].y,12,16), lsb_to_mps2(bmi08_accel[idx].z,12,16));
                            }

                            rslt = bmi08a_get_sensor_time(&bmi08dev, &sensor_time);
                            bmi08_error_codes_print_result("bmi08a_get_sensor_time", rslt);

                            printf("Sensor time : %.4lf   s\n", (sensor_time * BMI08_SENSORTIME_RESOLUTION));
                        }
                        if(true){
                            gyro_length = BMI08_FIFO_EXTRACTED_DATA_FRAME_COUNT;

                            rslt = bmi08g_get_fifo_config(&gyr_conf, &bmi08dev);
                            bmi08_error_codes_print_result("bmi08g_get_fifo_config", rslt);

                            rslt = bmi08g_get_fifo_length(&gyr_conf, &fifo);
                            bmi08_error_codes_print_result("bmi08g_get_fifo_length", rslt);

                            /* Read FIFO data */
                            rslt = bmi08g_read_fifo_data(&fifo, &bmi08dev);
                            bmi08_error_codes_print_result("bmi08g_read_fifo_data", rslt);

                            printf("Gyro FIFO buffer size : %d\n", BMI08_FIFO_RAW_DATA_BUFFER_SIZE);
                            printf("Gyro FIFO length available : %d\n\n", fifo.length);

                            printf("Requested data frames before parsing: %d\n", gyro_length);

                            /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
                            bmi08g_extract_gyro(bmi08_gyro, &gyro_length, &gyr_conf, &fifo);
                            bmi08_error_codes_print_result("bmi08g_extract_gyro", rslt);

                            printf("Parsed gyroscope frames: %d\n", gyr_conf.frame_count);

                            printf("\nFrame_Count, X, Y, Z\n");

                            /* Print the parsed gyroscope data from the FIFO buffer */
                            for (idx = 0; idx < gyr_conf.frame_count; idx++)
                            {
                                //printf("%d, %d, %d, %d\n", idx, bmi08_gyro[idx].x, bmi08_gyro[idx].y, bmi08_gyro[idx].z);
                            }
                        }

                    attempt++;
                }
            }
        }
    }
}
