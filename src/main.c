#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <string.h>
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include <math.h>

#include "common.h"
#include "bmp5.h"

#include "bmi08x.h"
#include "commonBMI08X.h"

/************************************************************************/
/*********                    BMP Macros                           ******/
/************************************************************************/
#define LOOP_COUNT                  UINT8_C(20)
#define THRESHOLD_LEVEL             UINT8_C(8)
#define BMP5_FIFO_DATA_BUFFER_SIZE  UINT8_C(96)
#define BMP5_FIFO_DATA_USER_LENGTH  UINT8_C(96)
#define BMP5_FIFO_P_T_FRAME_COUNT   UINT8_C(16)
#define BMP5_FIFO_T_FRAME_COUNT     UINT8_C(32)
#define BMP5_FIFO_P_FRAME_COUNT     UINT8_C(32)

/* Defines frame count requested
 * As, only Pressure is enabled in this example,
 * Total byte count requested : FIFO_FRAME_COUNT * BMP3_LEN_P_OR_T_HEADER_DATA
 */
#define FIFO_FRAME_COUNT  UINT8_C(50)

/* Defines watermark level of frame count requested
 * As, Pressure and Temperature are enabled in this example,
 * Total byte count requested : FIFO_WATERMARK_FRAME_COUNT * BMP3_LEN_P_AND_T_HEADER_DATA
 */
#define FIFO_WATERMARK_FRAME_COUNT  UINT8_C(5)

/* Maximum FIFO size */
#define FIFO_MAX_SIZE     UINT16_C(512)

#define PRESSURE_READING_FREQ   100
#define TASK_1_FREQ             100
#define ACCEL_READING_FREQ      100

#define BMP5_THRESHOLD_LEVEL 10

/*********************************************************************/
/*                              BMI Macros                           */
/*********************************************************************/

/* Buffer size allocated to store raw FIFO data for accel */
#define BMI08_ACC_FIFO_RAW_DATA_BUFFER_SIZE           UINT16_C(1024)

/* Length of data to be read from FIFO for accel */
#define BMI08_ACC_FIFO_RAW_DATA_USER_LENGTH           UINT16_C(1024)

/* Watermark level for accel */
#define BMI08_ACC_FIFO_WATERMARK_LEVEL                UINT16_C(10)

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

static QueueHandle_t xQueue = NULL;



//Presure function definitions
// Constants
#define P0 101325.0    // Standard atmospheric pressure at sea level in Pa
#define T0 288.15      // Standard temperature at sea level in K
#define L 0.0065       // Temperature lapse rate in K/m
#define R 287.05       // Specific gas constant for dry air in J/(kg·K)
#define G 9.80665      // Acceleration due to gravity in m/s²

// Function to calculate altitude
double calculateAltitude(double pressure) {
    return 44330.0 * (1.0 - pow((pressure / P0), (1.0 / 5.255)));
}

/* Define a variable to hold the initial timer count. */
static uint32_t start_time;

/* Configure Timer 1 to generate the runtime statistics counter. */
void configureTimerForRunTimeStats(void) {
    /* Store the initial count of the microsecond timer. */
    start_time = time_us_32();
}

/* Get the current value of the runtime counter. */
unsigned long getRunTimeCounterValue(void) {
    /* Return the elapsed time in microseconds since start_time. */
    return time_us_32() - start_time;
}

void printRunTimeStats(void) {
    char runtimeStatsBuffer[1024];
    memset(runtimeStatsBuffer, 0, sizeof(runtimeStatsBuffer));
    
    // Get runtime statistics
    vTaskGetRunTimeStats(runtimeStatsBuffer);

    // Print the runtime statistics
    printf("Task Name\t\tRuntime\t\tPercentage\n");
    printf("---------\t\t-------\t\t----------\n");

    unsigned long totalRuntime = getRunTimeCounterValue();
    char *line = strtok(runtimeStatsBuffer, "\n");
    while (line != NULL) {
        char taskName[configMAX_TASK_NAME_LEN];
        unsigned long runtime;

        // Parse the line to extract task name and runtime
        if (sscanf(line, "%s %lu", taskName, &runtime) == 2) {
            // Calculate percentage
            float percentage = (runtime * 100.0) / totalRuntime;
            printf("%s\t\t%lu\t\t%.2f%%\n", taskName, runtime, percentage);
        }

        line = strtok(NULL, "\n");
    }
}


void runtime_stats_task(void *pvParameters) {
    while (true) {
        printRunTimeStats();
        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
    }
}



void task1(void *pvParameters){

    //Code to run task every .1 seconds
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = TASK_1_FREQ; //every 100 ticks run this thread

    // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();

    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        printf("Task 1 is currently running\n");
    }

}

static int8_t set_config(struct bmp5_fifo *fifo, struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_osr_odr_press_config osr_odr_press_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 50Hz */
            osr_odr_press_cfg.odr = BMP5_ODR_100_2_HZ;

            /* Enable pressure */
            osr_odr_press_cfg.press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            osr_odr_press_cfg.osr_t = BMP5_OVERSAMPLING_8X;
            osr_odr_press_cfg.osr_p = BMP5_OVERSAMPLING_64X;

            rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);

            if (rslt == BMP5_OK)
            {
                set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
                set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;

                rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
                bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_fifo_configuration(fifo, dev);
            bmp5_error_codes_print_result("bmp5_get_fifo_configuration", rslt);

            if (rslt == BMP5_OK)
            {
                fifo->mode = BMP5_FIFO_MODE_STREAMING;

                /* Frame selection can be used to select data frames,
                 * pressure data only(32 frames) - BMP5_FIFO_PRESSURE_DATA
                 * temperature data only(32 frames) - BMP5_FIFO_TEMPERATURE_DATA
                 * both pressure and temperature data(16 frames) - BMP5_FIFO_PRESS_TEMP_DATA
                 * Here, both pressure and temperature data is selected(BMP5_FIFO_PRESS_TEMP_DATA)
                 */
                fifo->frame_sel = BMP5_FIFO_PRESS_TEMP_DATA;
                fifo->dec_sel = BMP5_FIFO_NO_DOWNSAMPLING;
                fifo->threshold = BMP5_THRESHOLD_LEVEL;
                fifo->set_fifo_iir_t = BMP5_ENABLE;
                fifo->set_fifo_iir_p = BMP5_ENABLE;

                rslt = bmp5_set_fifo_configuration(fifo, dev);
                bmp5_error_codes_print_result("bmp5_set_fifo_configuration", rslt);
            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);
            bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.fifo_thres_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /*
         * FIFO example can be executed on,
         * normal mode - BMP5_POWERMODE_NORMAL
         * continuous mode - BMP5_POWERMODE_CONTINOUS
         * Here, used normal mode (BMP5_POWERMODE_NORMAL)
         */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode2", rslt);
    }

    return rslt;
}

static int8_t get_fifo_data(struct bmp5_fifo *fifo, struct bmp5_dev *dev)
{
    int8_t rslt = 0;
    uint8_t idx = 0;
    uint8_t loop = 0;
    uint8_t int_status;
    uint8_t fifo_buffer[BMP5_FIFO_DATA_BUFFER_SIZE];
    struct bmp5_sensor_data sensor_data[BMP5_FIFO_P_T_FRAME_COUNT] = { { 0 } };


    //WRONG - EITHER DO INTERRUPT OR POLL - NOT BOTH
    //Code to run task every .1 seconds
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = PRESSURE_READING_FREQ; //every 100 ticks run this thread
    BaseType_t xWasDelayed; //used to notice if the function took too long to execute

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {

        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        if(xWasDelayed==pdFALSE){
            printf("****PRESSURE TASK WAS DELAYED****");
        }

        rslt = bmp5_get_interrupt_status(&int_status, dev);
        bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);

        //if (int_status & BMP5_INT_ASSERTED_FIFO_THRES)
        if(true)
        {
            fifo->length = BMP5_FIFO_DATA_USER_LENGTH;
            fifo->data = fifo_buffer;

            printf("\nIteration: %d\n", loop);
            printf("Each fifo frame contains 6 bytes of data\n");
            printf("Fifo data bytes requested: %d\n", fifo->length);

            rslt = bmp5_get_fifo_data(fifo, dev);
            bmp5_error_codes_print_result("bmp5_get_fifo_data", rslt);

            printf("Fifo data bytes available: %d\n", fifo->length);
            printf("Fifo threshold level : %d\n", fifo->threshold);
            printf("Fifo frames available: %d\n", fifo->fifo_count);

            if (rslt == BMP5_OK)
            {
                rslt = bmp5_extract_fifo_data(fifo, sensor_data);
                bmp5_error_codes_print_result("bmp5_extract_fifo_data", rslt);

                if (rslt == BMP5_OK)
                {
                    printf("\nData, Pressure (Pa), Altitude (m), Temperature (deg C)\n");

                    for (idx = 0; idx < fifo->fifo_count; idx++)
                    {
#ifdef BMP5_USE_FIXED_POINT
                        printf("%d, %lu, %ld\n",
                               idx,
                               (long unsigned int)sensor_data[idx].pressure,
                               (long int)sensor_data[idx].temperature);
#else
                        printf("%d, %f, %f, %f\n", idx, sensor_data[idx].pressure, calculateAltitude(sensor_data[idx].pressure), sensor_data[idx].temperature);
#endif
                    }
                }
            }

            loop++;
        }
    }

    return rslt;
}


void BMP5PressureWaterMark(void *pvParameters){
    int8_t rslt;
    struct bmp5_dev dev;
    struct bmp5_fifo fifo;

    /* Interface reference is given as a parameter
     * For I2C : BMP5_I2C_INTF
     * For SPI : BMP5_SPI_INTF
     */
    rslt = bmp5_interface_init(&dev, BMP5_I2C_INTF);
    bmp5_error_codes_print_result("bmp5_interface_init", rslt);

    if (rslt == BMP5_OK)
    {
        rslt = bmp5_init(&dev);
        bmp5_error_codes_print_result("bmp5_init", rslt);

        if (rslt == BMP5_OK)
        {
            rslt = set_config(&fifo, &dev);
            bmp5_error_codes_print_result("set_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            rslt = get_fifo_data(&fifo, &dev);
            bmp5_error_codes_print_result("get_fifo_data", rslt);
        }
    }
}

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
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;

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
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_250_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;
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

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMI08_ACC_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Number of accelerometer frames */
    uint16_t accel_length = BMI08_ACC_FIFO_WM_EXTRACTED_DATA_FRAME_COUNT;

    /* Variable to index bytes */
    uint16_t idx = 0;

    uint8_t try = 1;

    /* Variable to store sensor time value */
    uint32_t sensor_time;

    /* Variable to store available fifo length */
    uint16_t fifo_length;

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
        //bmi08a_soft_reset(&bmi08dev);

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
                    printf("****ACCELERATION TASK WAS DELAYED****");
                }

                rslt = bmi08a_get_data_int_status(&status, &bmi08dev);
                bmi08_error_codes_print_result("bmi08a_get_data_int_status", rslt);

                if (status & BMI08_ACCEL_FIFO_WM_INT)
                {
                    printf("\nIteration : %d\n", try);

                    /* Update FIFO structure */
                    fifo_frame.data = fifo_data;
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

                    if (rslt == BMI08_OK)
                    {
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
                            printf("%d, %f, %f, %f\n", idx, lsb_to_mps2(bmi08_accel[idx].x,12,16), lsb_to_mps2(bmi08_accel[idx].y,12,16), lsb_to_mps2(bmi08_accel[idx].z,12,16));
                        }

                        rslt = bmi08a_get_sensor_time(&bmi08dev, &sensor_time);
                        bmi08_error_codes_print_result("bmi08a_get_sensor_time", rslt);

                        printf("Sensor time : %.4lf   s\n", (sensor_time * BMI08_SENSORTIME_RESOLUTION));
                    }

                    try++;
                }
            }
        }
    }
}

int main(){
    stdio_init_all();

    printf("program beginning\n");

    configureTimerForRunTimeStats();

    xQueue = xQueueCreate(1,sizeof(uint));

    xTaskCreate(task1,"Task1", 256, NULL, 1, NULL);
    //xTaskCreate(task2,"Pressure", 8192, NULL, 10, NULL);
    //xTaskCreate(BMPPressureWaterMark,"Pressure", 8192, NULL, 10, NULL);
    //xTaskCreateAffinitySet(BMP5PressureWaterMark, "Pressure", 8192, NULL, 10, 1, NULL);
    xTaskCreateAffinitySet(BMI_ACC_FIFO, "Acceleration", 8192, NULL, 10, 1, NULL);
    xTaskCreate(runtime_stats_task, "Runtime_Stats_Task", 1024, NULL, 3, NULL);

    vTaskStartScheduler();

    while(1){};
}