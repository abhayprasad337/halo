/*
 * motion_analysis.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: unnik
 */

//#define DEBUG_E
#define VERBOSE_E
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
#define TEMP_LOGGER_SZ 40
#include "eint3_monitoring_hw_int.hpp"
#include "io.hpp"
#include "storage.hpp"
#define DUMP_FILE_NAME "0:log_%u.csv"
#define DUMP_SW_FILE_NAME "0:log_%u_sw.csv"
//remember: each log currently is 12 bytes worst case; so 30 minutes with each reading 100ms apart will be 351KB
//#define DURATION_TO_LOG_MS (((uint64_t)(1)) * 10 * 1000)
#define DURATION_TO_LOG_MS (((uint64_t)(30)) * 60 * 1000) //30 minutes
#define DEBOUNCE_SW_INTERVAL_MS (1000)
#endif /**< DUMP_ACC_VALUES */

#define DELAY_BETWEEN_ACC_READS_TICKS 10 //100ms for SJ One

//tick is RTOS System Tick Interrupt
//configTICK_RATE_HZ; //the number of ticks in 1 second
//portTICK_PERIOD_MS; //each tick shall happen in these many ms
//So our System Tick Interrupt is 1ms configTICK_RATE_HZ=1000 (which is 1 tick in 1ms)

typedef struct
{
    TaskHandle_t xRdAcc;
#ifdef DUMP_ACC_VALUES
    uint32_t ulFileIdx;
    SoftTimer durationToLog;
    char pcDumpFileName[40];
    char pcDumpFileNameSW[40];
    SoftTimer debounceTimerSW;
    bool swLit;
#endif /**< DUMP_ACC_VALUES */
}tMotionAnalysis;

#define THRESHOLD_AXIS_VALUE 150
void taskReadAS(void* p)
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)p;
    int16_t x=0,y=0;
    while(1)
    {
        x = AS.getX();
        y = AS.getY();
#ifdef DUMP_ACC_VALUES

        char pcLog[TEMP_LOGGER_SZ] = {0};
        int nCountL;
        FRESULT fr;
        if(!pxMA->durationToLog.expired())
        {
            nCountL = snprintf(pcLog, TEMP_LOGGER_SZ, "%d %d %d\n", (int)SoftTimer::getCurrentTimeMs(), x, y);
            LOGD("writing %s %d\n", pcLog, nCountL);
            fr = Storage::append(pxMA->pcDumpFileName, pcLog, nCountL, 0 /** append to end; create new file if req */);
            if(fr != FR_OK)
            {
                LPC_GPIO1->FIOCLR = (1 << 8);
                LOGD("append failed\n");
            }
        }
#endif /**< DUMP_ACC_VALUES */
        vTaskDelay(DELAY_BETWEEN_ACC_READS_TICKS);
    }
}
void initMA(tMotionAnalysis* pxMA)
{
#ifdef DUMP_ACC_VALUES
    pxMA->ulFileIdx = 0;;
    pxMA->durationToLog = SoftTimer(DURATION_TO_LOG_MS);
    pxMA->debounceTimerSW = SoftTimer(DEBOUNCE_SW_INTERVAL_MS);
    snprintf(pxMA->pcDumpFileName, 40, DUMP_FILE_NAME, (unsigned)(pxMA->ulFileIdx));
    snprintf(pxMA->pcDumpFileNameSW, 40, DUMP_SW_FILE_NAME, (unsigned)(pxMA->ulFileIdx));
    LOGV("opening file [%s]\n", pxMA->pcDumpFileName);
    //creating new file using Storage::write() so as to overwrite any extant file by same name
    Storage::write(pxMA->pcDumpFileName, NULL, 0, 0);
    Storage::write(pxMA->pcDumpFileNameSW, NULL, 0, 0);
#if 0
    gxSWINTHWSem = xSemaphoreCreateBinary();
    if(!gxSWINTHWSem)
        return;
    pxMA->xMx = xSemaphoreCreateMutex();
#endif
#endif /**< DUMP_ACC_VALUES */
}

#ifdef DUMP_ACC_VALUES
void user_EINT3_callback_dump_ts(tPINInfo* apPinInfo)
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)apPinInfo->pUserData;
    if(pxMA->debounceTimerSW.expired())
    {
        char pcLog[TEMP_LOGGER_SZ] = {0};
        /** dump the switch press timestamp */
        int nCountL = snprintf(pcLog, TEMP_LOGGER_SZ, "%d\n", (int)(apPinInfo->ullTS));
        LOGV("switch pressed\n");
        if(pxMA->swLit)
            LPC_GPIO1->FIOSET = (1 << 4);
        else
            LPC_GPIO1->FIOCLR = (1 << 4);
        pxMA->swLit = !pxMA->swLit;
        FRESULT fr = Storage::append(pxMA->pcDumpFileNameSW, pcLog, nCountL, 0 /** append to end; create new file if req */);
        if(fr != FR_OK)
        {
            LPC_GPIO1->FIOCLR = (1 << 8);
            LOGD("append SW failed\n");
        }
        pxMA->debounceTimerSW.reset(DEBOUNCE_SW_INTERVAL_MS);
    }
}
#endif /**< DUMP_ACC_VALUES */

void* xStartMotionAnalysis()
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)calloc(1, sizeof(tMotionAnalysis));
    if(!pxMA)
        return NULL;
    initMA(pxMA);
    xTaskCreate(taskReadAS, "rdAS_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);
    //xTaskCreate(taskSwitchListener, "swL_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);

#ifdef DUMP_ACC_VALUES
    /** start the EINT3 handler to sense key presses on P0.1 */
    {
        tPINInfo pinInfo;
        eint3_monitoring_hw_int* pEINT3Monitor;
        scheduler_add_task(pEINT3Monitor = new eint3_monitoring_hw_int());
        /**
         * We will attach switch to P0.1
         * This P0.1 shall be configured as INPUT - GPIO pins */
        /**
         * a) PINSEL is by default 0, so PINS are GPIO by def
         * b) FIODIR - set GPIO PIN direction as input
         * c) PINMODE - set GPIO PIN mode to use pull-down resistor so that
         * we can connect Vcc to switch; switch to GPIO
         * */
        const uint32_t uPIN1Dir = (1 << 1);
        LPC_GPIO0->FIODIR &= ~uPIN1Dir; //0 means corresp pin is input
        /*on-chip pull-down resistor enabled - 2'b11
         * P0.1: (3:2) PINMODE0
         * */
        const uint32_t uPin1ModePullDown =  (3 << 2);
        LPC_PINCON->PINMODE0 |= uPin1ModePullDown;
        pinInfo.nPort = eint3_monitor_PORT0;
        pinInfo.nPIN = 1;
        pinInfo.nAssociatedINTTypes = kINT_EdgeTrig_R;
        pinInfo.cb = user_EINT3_callback_dump_ts;
        pinInfo.pUserData = (void*)pxMA;
        pEINT3Monitor->RegisterINTCallback(&pinInfo);
    }
#endif /**< DUMP_ACC_VALUES */

    return (void*)pxMA;
}
