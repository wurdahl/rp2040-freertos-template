#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <string.h>
#include "hardware/timer.h"
#include <math.h>

#include "common.h"
#include "bmp5.h"

#include "bmi08x.h"
#include "commonBMI08X.h"

#include "i2cLib.h"

//mutex for protecting print commands
static SemaphoreHandle_t printMutex = NULL;

//This includes all of the helper functions and the task function for sampling pressure
#include "pressure.h"
//This includes all of th helper functions and the task function for sampling acceleration
#include "accel.h"
//This includes all of the helper functions and the task function for reporting CPU usage stats
#include "runtimeStats.h"
//This includes all of the helper functions and the task functions for sampling the magnetometer
#include "mag.h"

static QueueHandle_t xQueue = NULL;

#define TASK_1_FREQ 100

void task1(void *pvParameters){

    //Code to run task every .1 seconds
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = TASK_1_FREQ; //every 100 ticks run this thread

    // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();

    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        xSemaphoreTake( printMutex, portMAX_DELAY );
        {
            printf("Task 1 is currently running\n");
        }
        xSemaphoreGive( printMutex );
    }

}

int main(){
    stdio_init_all();

    printf("program beginning\n");

    printMutex = xSemaphoreCreateMutex();

    configureTimerForRunTimeStats();

    xQueue = xQueueCreate(1,sizeof(uint));

    init_i2cLib();

    xTaskCreate(task1,"Task1", 256, NULL, 1, NULL);
    xTaskCreate(BMI_ACC_FIFO,"Acceleration",8192, NULL, 11, NULL);
    xTaskCreate(BMP5PressureWaterMark,"Pressure", 8192, NULL, 10, NULL);
    xTaskCreate(magPolling, "Magnetometer", 8192, NULL,9,NULL);
    xTaskCreate(runtime_stats_task, "Runtime_Stats_Task", 1024, NULL, 15, NULL);

    //If using multicore, then use these functions to start the sampling tasks
    //xTaskCreateAffinitySet(BMP5PressureWaterMark, "Pressure", 8192, NULL, 11, 1, NULL);
    //xTaskCreateAffinitySet(BMI_ACC_FIFO, "Acceleration", 8192, NULL, 10, 1, NULL);

    vTaskStartScheduler();

    while(1){};
}