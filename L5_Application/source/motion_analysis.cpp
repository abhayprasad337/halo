/*
 * motion_analysis.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: unnik
 */

//#define DEBUG_E
//#define VERBOSE_E
#include "debug_e.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart0_min.h"
#include "motion_analysis.hpp"
#include "io.hpp" //for AS
#include "printf_lib.h"
#include "stdlib.h"
#include "stdio.h"
#include "soft_timer.hpp"

#define DUMP_ACC_VALUES
#ifdef DUMP_ACC_VALUES
#include "io.hpp"
#include "storage.hpp"
#define DUMP_FILE_NAME "1:log_%u.csv"
//remember: each log currently is 12 bytes worst case; so 30 minutes with each reading 100ms apart will be 351KB
//#define DURATION_TO_LOG_MS (((uint64_t)(1)) * 10 * 1000)
#define DURATION_TO_LOG_MS (((uint64_t)(30)) * 60 * 1000) //30 minutes
#endif
#define DELAY_BETWEEN_ACC_READS_TICKS 10 //100ms for SJ One

//tick is RTOS System Tick Interrupt
//configTICK_RATE_HZ; //the number of ticks in 1 second
//portTICK_PERIOD_MS; //each tick shall happen in these many ms
//So our System Tick Interrupt is 1ms configTICK_RATE_HZ=1000 (which is 1 tick in 1ms)

typedef struct
{
    TaskHandle_t xRdAcc;
    SoftTimer durationToLog;
    uint32_t ulFileIdx;
    SemaphoreHandle_t xMx;
    char pcDumpFileName[40];
}tMotionAnalysis;

#define THRESHOLD_AXIS_VALUE 150
void taskReadAS(void* p)
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)p;
    int16_t x=0,y=0;
    #define TEMP_LOGGER_SZ 40
    char pcLog[TEMP_LOGGER_SZ] = {0};
    int nCountL;
    FRESULT fr;
    while(1)
    {
        x = AS.getX();
        y = AS.getY();
#ifdef DUMP_ACC_VALUES
        if(!pxMA->durationToLog.expired())
        {
            nCountL = snprintf(pcLog, TEMP_LOGGER_SZ, "%d %d %d\n", (int)SoftTimer::getCurrentTimeMs(), x, y);
            LOGV("writing %s %d\n", pcLog, nCountL);
            fr = Storage::append(pxMA->pcDumpFileName, pcLog, nCountL, 0 /** append to end; create new file if req */);
            if(fr != FR_OK)
            {
                LPC_GPIO1->FIOCLR = (1 << 8);
                LOGE("append failed\n");
            }
        }
#endif /**< DUMP_ACC_VALUES */
        vTaskDelay(DELAY_BETWEEN_ACC_READS_TICKS);
    }
}
#if 0
static SemaphoreHandle_t gxSWINTHWSem;
static SoftTimer gTimerSWINT(50);
void handleSWINTHWSem()
{
    BaseType_t xHigherPriorityTaskWoken;
    LOGD("INTERRUPTED\n");
    if(gTimerSWINT.expired())
    {
        if(xSemaphoreGiveFromISR(gxSWINTHWSem, &xHigherPriorityTaskWoken))
        {
            gTimerSWINT.reset(500);
            if(xHigherPriorityTaskWoken == pdTRUE)
            {
                LOGD("higher priority task woken; do context switch\n");
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                LOGD("yield from isr done\n");
            }
            else
            {
                LOGD("no task got woken up with the SemaphoreGive\n");
            }
        }
        else
        {
            LOGD("xSemaphoreGiveFromISR failed\n");
        }
    }
}
#endif
void initMA(tMotionAnalysis* pxMA)
{
#ifdef DUMP_ACC_VALUES
    pxMA->ulFileIdx = 0;;
    pxMA->durationToLog = SoftTimer(DURATION_TO_LOG_MS);
    snprintf(pxMA->pcDumpFileName, 40, DUMP_FILE_NAME, (unsigned)(pxMA->ulFileIdx));
    //creating new file using Storage::write() so as to overwrite any extant file by same name
    Storage::write(pxMA->pcDumpFileName, NULL, 0, 0);
#if 0
    gxSWINTHWSem = xSemaphoreCreateBinary();
    if(!gxSWINTHWSem)
        return;
    pxMA->xMx = xSemaphoreCreateMutex();
#endif
#endif /**< DUMP_ACC_VALUES */
}
#if 0
void taskSwitchListener(void* p)
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)p;
    while(1){
        if(xSemaphoreTake(gxSWINTHWSem, portMAX_DELAY) == pdTRUE)
        {
            LOGD("Switch pressed @ P2.2!!\n");
            // pxMA->xMx
        }
        else
        {
            LOGD("semaphore take failed\n");
        }
    }
}
#endif
void* xStartMotionAnalysis()
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)calloc(1, sizeof(tMotionAnalysis));
    if(!pxMA)
        return NULL;
    initMA(pxMA);
    xTaskCreate(taskReadAS, "rdAS_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);
    //xTaskCreate(taskSwitchListener, "swL_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);
    return (void*)pxMA;
}
