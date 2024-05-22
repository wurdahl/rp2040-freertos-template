#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <string.h>

#include "bmp3.h"
#include "common.h"

/************************************************************************/
/*********                     Macros                              ******/
/************************************************************************/

/* Defines frame count requested
 * As, only Pressure is enabled in this example,
 * Total byte count requested : FIFO_FRAME_COUNT * BMP3_LEN_P_OR_T_HEADER_DATA
 */
#define FIFO_FRAME_COUNT  UINT8_C(50)

/* Defines watermark level of frame count requested
 * As, Pressure and Temperature are enabled in this example,
 * Total byte count requested : FIFO_WATERMARK_FRAME_COUNT * BMP3_LEN_P_AND_T_HEADER_DATA
 */
#define FIFO_WATERMARK_FRAME_COUNT  UINT8_C(10)

/* Maximum FIFO size */
#define FIFO_MAX_SIZE     UINT16_C(512)

#define PRESSURE_READING_FREQ   100
#define TASK_1_FREQ             100

static QueueHandle_t xQueue = NULL;

#include "hardware/timer.h"
#include "pico/stdlib.h"

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

void task2(void *pvParameters){

    //BMP 390 Stuff
    struct bmp3_dev dev;
    int8_t rslt;

    uint8_t try = 1;
    uint8_t index = 0;
    uint16_t settings_sel;
    uint16_t settings_fifo;
    uint16_t fifo_length = 0;
    uint8_t fifo_data[FIFO_MAX_SIZE];
    struct bmp3_data fifo_p_t_data[FIFO_MAX_SIZE];
    struct bmp3_fifo_settings fifo_settings = { 0 };
    struct bmp3_settings settings = { 0 };
    struct bmp3_fifo_data fifo = { 0 };
    struct bmp3_status status = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    /*
     * NOTE : Temperature will be enabled, to get pressure data.
     */

    fifo_settings.mode = BMP3_ENABLE;
    fifo_settings.press_en = BMP3_ENABLE;
    fifo_settings.filter_en = BMP3_ENABLE;
    fifo_settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;
    fifo_settings.ffull_en = BMP3_ENABLE;

    fifo.buffer = fifo_data;
    fifo.req_frames = FIFO_FRAME_COUNT;

    settings.press_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_ODR;

    settings_fifo = BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_PRESS_EN | BMP3_SEL_FIFO_FULL_EN | BMP3_SEL_FIFO_DOWN_SAMPLING |
                    BMP3_SEL_FIFO_FILTER_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    rslt = bmp3_set_fifo_settings(settings_fifo, &fifo_settings, &dev);
    bmp3_check_rslt("bmp3_set_fifo_settings", rslt);

    rslt = bmp3_get_fifo_settings(&fifo_settings, &dev);
    bmp3_check_rslt("bmp3_get_fifo_settings", rslt);

    printf("Read Fifo full interrupt data\n");

    //Code to run task every .1 seconds
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = PRESSURE_READING_FREQ; //every 100 ticks run this thread

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while(true){

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        if ((rslt == BMP3_OK) && (status.intr.fifo_full == BMP3_ENABLE))
        {
            rslt = bmp3_get_fifo_length(&fifo_length, &dev);
            bmp3_check_rslt("bmp3_get_fifo_length", rslt);

            rslt = bmp3_get_fifo_data(&fifo, &fifo_settings, &dev);
            bmp3_check_rslt("bmp3_get_fifo_data", rslt);

            /* NOTE : Read status register again to clear FIFO Full interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            if (rslt == BMP3_OK)
            {
                printf("\nIteration : %d\n", try);
                printf("Available fifo length : %d\n", fifo_length);
                printf("Fifo byte count from fifo structure : %d\n", fifo.byte_count);

                printf("FIFO frames requested : %d\n", fifo.req_frames);

                rslt = bmp3_extract_fifo_data(fifo_p_t_data, &fifo, &dev);
                bmp3_check_rslt("bmp3_extract_fifo_data", rslt);

                printf("FIFO frames extracted : %d\n", fifo.parsed_frames);

                for (index = 0; index < fifo.parsed_frames; index++)
                {
                        #ifdef BMP3_FLOAT_COMPENSATION
                    printf("Frame[%d]  T: %.2f deg C, P: %.2f Pa\n", index, (fifo_p_t_data[index].temperature),
                           (fifo_p_t_data[index].pressure));
                        #else
                    printf("Frame[%d]  T: %ld deg C, P: %lu Pa\n", index,
                           (long int)(int32_t)(fifo_p_t_data[index].temperature / 100),
                           (long unsigned int)(uint32_t)(fifo_p_t_data[index].pressure / 100));
                        #endif
                }
            }

        }

    }
}

void BMPPressureWaterMark(void *pvParameters){
    struct bmp3_dev dev;
    int8_t rslt;

    uint16_t settings_sel;
    uint16_t settings_fifo;
    uint8_t loop = 1;
    uint8_t index = 0;
    uint16_t watermark = 0;
    uint16_t fifo_length = 0;
    uint8_t fifo_data[FIFO_MAX_SIZE];
    struct bmp3_data fifo_p_t_data[FIFO_MAX_SIZE];
    struct bmp3_fifo_settings fifo_settings = { 0 };
    struct bmp3_settings settings = { 0 };
    struct bmp3_fifo_data fifo = { 0 };
    struct bmp3_status status = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    fifo_settings.mode = BMP3_ENABLE;
    fifo_settings.press_en = BMP3_ENABLE;
    fifo_settings.temp_en = BMP3_ENABLE;
    fifo_settings.filter_en = BMP3_ENABLE;
    fifo_settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;
    fifo_settings.fwtm_en = BMP3_ENABLE;
    fifo_settings.time_en = BMP3_ENABLE;

    fifo.buffer = fifo_data;
    fifo.req_frames = FIFO_WATERMARK_FRAME_COUNT;

    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;
    settings.int_settings.latch = BMP3_ENABLE;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;

    settings_fifo = BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_PRESS_EN | BMP3_SEL_FIFO_TEMP_EN | BMP3_SEL_FIFO_TIME_EN |
                    BMP3_SEL_FIFO_FWTM_EN | BMP3_SEL_FIFO_DOWN_SAMPLING | BMP3_SEL_FIFO_FILTER_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    rslt = bmp3_set_fifo_watermark(&fifo, &fifo_settings, &dev);
    bmp3_check_rslt("bmp3_set_fifo_watermark", rslt);

    rslt = bmp3_set_fifo_settings(settings_fifo, &fifo_settings, &dev);
    bmp3_check_rslt("bmp3_set_fifo_settings", rslt);

    rslt = bmp3_get_fifo_settings(&fifo_settings, &dev);
    bmp3_check_rslt("bmp3_get_fifo_settings", rslt);

    printf("Read Fifo watermark interrupt data with sensor time\n");

    //Code to run task every .1 seconds
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = PRESSURE_READING_FREQ; //every 100 ticks run this thread

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (true){

        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        rslt = bmp3_get_status(&status, &dev);
        bmp3_check_rslt("bmp3_get_status", rslt);

        if ((rslt == BMP3_OK) && (status.intr.fifo_wm == BMP3_ENABLE))
        {
            rslt = bmp3_get_fifo_length(&fifo_length, &dev);
            bmp3_check_rslt("bmp3_get_fifo_length", rslt);

            rslt = bmp3_get_fifo_watermark(&watermark, &dev);
            bmp3_check_rslt("bmp3_get_fifo_watermark", rslt);

            rslt = bmp3_get_fifo_data(&fifo, &fifo_settings, &dev);
            bmp3_check_rslt("bmp3_get_fifo_data", rslt);

            /* NOTE : Read status register again to clear FIFO watermark interrupt status */
            rslt = bmp3_get_status(&status, &dev);
            bmp3_check_rslt("bmp3_get_status", rslt);

            if (rslt == BMP3_OK)
            {
                rslt = bmp3_extract_fifo_data(fifo_p_t_data, &fifo, &dev);
                bmp3_check_rslt("bmp3_extract_fifo_data", rslt);

                printf("\nIteration : %d\n", loop);
                printf("Watermark level : %d\n", watermark);
                printf("Available fifo length : %d\n", fifo_length);
                printf("Fifo byte count from fifo structure : %d\n", fifo.byte_count);

                printf("FIFO compensation Pressure and Temperature frames requested : %d\n", fifo.req_frames);

                printf("FIFO frames extracted : %d\n", fifo.parsed_frames);

                for (index = 0; index < fifo.parsed_frames; index++)
                {
                    #ifdef BMP3_FLOAT_COMPENSATION
                    printf("Frame[%d]  T: %.2f deg C, P: %.2f Pa\n", index, (fifo_p_t_data[index].temperature),
                           (fifo_p_t_data[index].pressure));
                    #else
                    printf("Frame[%d]  T: %ld deg C, P: %lu Pa\n", index,
                           (long int)(int32_t)(fifo_p_t_data[index].temperature / 100),
                           (long unsigned int)(uint32_t)(fifo_p_t_data[index].pressure / 100));
                    #endif
                }

                printf("Sensor time : %lu\n", (long unsigned int)fifo.sensor_time);

                loop++;
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
    xTaskCreateAffinitySet(BMPPressureWaterMark, "Pressure", 8192, NULL, 10, 1, NULL);
    xTaskCreate(runtime_stats_task, "Runtime_Stats_Task", 1024, NULL, 3, NULL);

    vTaskStartScheduler();

    while(1){};
}