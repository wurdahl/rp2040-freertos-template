#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "hardware/timer.h"
#include <string.h>

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

    xSemaphoreTake( printMutex, portMAX_DELAY );
    {
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
    xSemaphoreGive( printMutex );
}


void runtime_stats_task(void *pvParameters) {
    while (true) {
        printRunTimeStats();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
    }
}