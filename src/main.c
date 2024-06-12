#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <string.h>

#include "common.h"
#include "bmp5.h"

/************************************************************************/
/*********                     Macros                              ******/
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

#define BMP5_THRESHOLD_LEVEL 10

static QueueHandle_t xQueue = NULL;

#include "hardware/timer.h"
#include "pico/stdlib.h"

//Presure function definitions
#include <math.h>
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

int main(){
    stdio_init_all();

    printf("program beginning\n");

    configureTimerForRunTimeStats();

    xQueue = xQueueCreate(1,sizeof(uint));

    xTaskCreate(task1,"Task1", 256, NULL, 1, NULL);
    //xTaskCreate(task2,"Pressure", 8192, NULL, 10, NULL);
    //xTaskCreate(BMPPressureWaterMark,"Pressure", 8192, NULL, 10, NULL);
    xTaskCreateAffinitySet(BMP5PressureWaterMark, "Pressure", 8192, NULL, 10, 1, NULL);
    xTaskCreate(runtime_stats_task, "Runtime_Stats_Task", 1024, NULL, 3, NULL);

    vTaskStartScheduler();

    while(1){};
}