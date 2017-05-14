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
#include "string.h"
#include "utilities.h"
#include "exp_halo_api.h"

/// #define DUMP_ACC_VALUES
#ifdef DUMP_ACC_VALUES
#define TEMP_LOGGER_SZ 40
#include "eint3_monitoring_hw_int.hpp"
#include "io.hpp"
#include "storage.hpp"
#define DUMP_FILE_NAME "1:log_%u.csv"
#define DUMP_SW_FILE_NAME "1:log_%u_sw.csv"
#define DUMP_CALIB_FILE_NAME "1:calib.csv"
/*
 remember: each log currently is 12 bytes worst case; so 30 minutes with each reading 100ms apart will be 351KB
 #define DURATION_TO_LOG_MS (((uint64_t)(1)) * 10 * 1000)
*/
#define DURATION_TO_LOG_MS (((uint64_t)(30)) * 60 * 1000) //30 minutes
#define DEBOUNCE_SW_INTERVAL_MS (1000)
#endif /**< DUMP_ACC_VALUES */

#define DELAY_BETWEEN_ACC_READS_TICKS 10 //100ms for SJ One

/*
 * tick is RTOS System Tick Interrupt
 * configTICK_RATE_HZ; //the number of ticks in 1 second
 * portTICK_PERIOD_MS; //each tick shall happen in these many ms
 * So our System Tick Interrupt is 1ms configTICK_RATE_HZ=1000 (which is 1 tick in 1ms)
*/

#define WINDOW_SIZE 100 // Size of window to perform moving average filtering
#define SECOND_WINDOW_SIZE 30 // Size of window to perform moving average filtering the second time
#define CAL_ARR_SIZE 3000 // Number of values required to calibrate -> 1 second of data


static int IS_CALIBRATE=1;
typedef struct
{
    TaskHandle_t xRdAcc;
    SoftTimer durationToLog;
#ifdef DUMP_ACC_VALUES
    uint32_t ulFileIdx;
    SoftTimer durationToLog;
    char pcDumpFileName[40];
    char pcDumpFileNameSW[40];
    SoftTimer debounceTimerSW;
    bool swLit;
#endif /**< DUMP_ACC_VALUES */
    int16_t sFirstReadings[WINDOW_SIZE];
	int16_t sReadings[SECOND_WINDOW_SIZE];
    int16_t sFirstSmoothenedValue;
    int16_t sSmoothenedValue;
    int16_t sCalReadings[CAL_ARR_SIZE];
    int16_t sCalibrationOffset;
    int16_t x_after_cal;

    tHalo_Ctx* xpHCtx;

}tMotionAnalysis;

#define THRESHOLD_AXIS_VALUE 150

void taskReadAS(void* p)
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)p;
    int16_t x=0,y=0;
	static int counter = 0;
    int16_t xVal;

    // Initialize moving_average_filter readings to 0.
    memset(&pxMA->sFirstReadings,0,sizeof(int16_t)*WINDOW_SIZE);
    memset(&pxMA->sReadings,0,sizeof(int16_t)*SECOND_WINDOW_SIZE);

#ifdef DUMP_ACC_VALUES
    char pcLog[TEMP_LOGGER_SZ] = {0};
    int nCountL;
    FRESULT fr;
#endif /**< DUMP_ACC_VALUES */


    /*
     * Parameters to control the state machine
     * which detects slow-stop-moving states.
     */



    volatile uint8_t state = 0; // Initializing the state to be in stop
    volatile uint8_t previous_state=0;

    volatile int16_t prev_acc = -1000;
    volatile int16_t acc_at_settled = 30;
    volatile int16_t hit_bottom_slow_acc= 0;
    volatile int16_t remove_from_stuck = 0;
    /*
     * Weak thresholds that help to
     * determine state of the subject.
     */
    int16_t T_ismoving = 20;
    int16_t T_isSlowDown = -10;
    int16_t T_isMovingFromSlowDown = 40;
    int16_t T_remov_from_stuck = 1000;
    /*
     * Confidence level parameters which
     * help in deciding the state transitions.
     */
    int16_t confidence_level_settling = 0;
    int16_t confidence_level_settled = 0;
    int16_t confidence_level_moving = 0;

    /*
     * Confidence thresholds
     */
    int16_t definitly_settling = 10;
    int16_t definitly_settled = 5;
    int16_t definitely_out_of_stop = 2;


    eHalo_Mod_MAE_Event myEvent;

    while(1)
    {

    	x = AS.getX();
    	if(!pxMA->durationToLog.expired())
    	{
#ifdef DUMP_ACC_VALUES
    		y = AS.getY();

    		if(!pxMA->durationToLog.expired())
    		{
    			nCountL = snprintf(pcLog, TEMP_LOGGER_SZ, "%d %d %d\n", (int)SoftTimer::getCurrentTimeMs(), x, y);
    			LOGD("writing %s %d\n", pcLog, nCountL);
    			fr = Storage::append(pxMA->pcDumpFileName, pcLog, nCountL, 0 /** append to end; create new file if req */);
    			if(fr != FR_OK)
    			{
    				LPC_GPIO1->FIOCLR = (1 << 8);
    				LOGE("append failed\n");
    			}
    		}
    		vTaskDelay(DELAY_BETWEEN_ACC_READS_TICKS);
#endif /**< DUMP_ACC_VALUES */




    		if (IS_CALIBRATE){

    			pxMA->sFirstSmoothenedValue = moving_average_filter(&(pxMA->sFirstReadings[0]),x,WINDOW_SIZE);


    			/// Calibration phase
    			/* Record N values and take the average of them */
    			pxMA->sCalReadings[counter]=pxMA->sFirstSmoothenedValue;
    			counter++;
    			LPC_GPIO1->FIOCLR = (1 << 0); // Calibration LED is ON
    			if(counter > CAL_ARR_SIZE){
    				pxMA->sCalibrationOffset = calibration(&(pxMA->sCalReadings[0]),CAL_ARR_SIZE);
    				LPC_GPIO1->FIOSET = (1 << 0); // turn off calibration LED
    				IS_CALIBRATE = 0;
    				LPC_GPIO1->FIOCLR = (1 << 1) | (1 << 4) | (1 << 8);
    				delay_ms(500);
    				LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 8) | (1 << 0);
#ifdef DUMP_ACC_VALUES // Store calibration value into file.
    				nCountL = snprintf(pcLog, TEMP_LOGGER_SZ, "%d\n", (int)(pxMA->sCalibrationOffet));
    				fr = Storage::write(DUMP_CALIB_FILE_NAME, pcLog, nCountL, 0 /** append to end; create new file if req */);
    				if(fr != FR_OK)
    				{
    					LPC_GPIO1->FIOCLR = (1 << 8);
    					LOGE("append failed\n");
    				}
#endif /**< DUMP_ACC_VALUES */
    			}
    		} else {

    			pxMA->x_after_cal = x - pxMA->sCalibrationOffset;
    			pxMA->sFirstSmoothenedValue = moving_average_filter(&(pxMA->sFirstReadings[0]),pxMA->x_after_cal,WINDOW_SIZE);
    			pxMA->sSmoothenedValue = moving_average_filter(&(pxMA->sReadings[0]),pxMA->sFirstSmoothenedValue,SECOND_WINDOW_SIZE);

    			/// Perform motion and slow down detection.
    			/// This has to be changed to pxMA->sSmoothenedValue.
    			/// This is because the algorithm thresholds have been set for this type of configuration.
    			xVal = pxMA->sSmoothenedValue;

    			switch(state){

    			case 0:
    				LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 8);
    				LPC_GPIO1->FIOCLR = (1 << 0);

    				/// STOP
    				/** Reset any previous state parameters */
    				confidence_level_settled = 0;
    				remove_from_stuck = 0;

    				/** In this state do this.. */
    				//u0_dbg_printf("STOP\n");

    				if((xVal < acc_at_settled+3) && (xVal > acc_at_settled-3)){
    					confidence_level_moving = 0;
    				} else {
    					confidence_level_moving = confidence_level_moving + 1;
    				}
    				prev_acc = xVal;


    				/** Condition for state transition */
    				if (confidence_level_moving > definitely_out_of_stop){
    					if  (xVal > acc_at_settled + 10){
    						state = 1;
    					}else{
    						if (xVal < T_isSlowDown){
    							state = 2;
    						}
    					}
    				}
    				myEvent = kHalo_Mod_MAE_EV_Stopped;
    				break;
    			case 1:
    				LPC_GPIO1->FIOSET = (1 << 0) | (1 << 4) | (1 << 8);
    				LPC_GPIO1->FIOCLR = (1 << 1);

    				/// MOVING
    				/** Reset any previous state parameters */
    				confidence_level_moving = 0;

    				/** In this state do this.. */
    				//u0_dbg_printf("MOVING\n");

    				/** Condition for state transition */
    				if (xVal < T_isSlowDown){
    					hit_bottom_slow_acc = 0;
    					prev_acc = xVal;
    					state = 2;
    				}

    				myEvent = kHalo_Mod_MAE_EV_Moving;
    				break;
    			case 2:
    				LPC_GPIO1->FIOSET = (1 << 1) | (1 << 0) | (1 << 8);
    				LPC_GPIO1->FIOCLR = (1 << 4);
    				/// SLOW-DOWN
    				/** Reset any previous state parameters */

    				/** In this state do this.. */
    				//u0_dbg_printf("SLOW_DOWN\n");

    				if (xVal > prev_acc){
    					confidence_level_settling = confidence_level_settling + 1;
    				}else {
    					confidence_level_settling = 0;
    				}
    				prev_acc = xVal;

    				/** Condition for state transition */
    				if (confidence_level_settling > definitly_settling){
    					hit_bottom_slow_acc = xVal;
    					state = 3;
    				}
    				myEvent = kHalo_Mod_MAE_EV_Stopping;
    				break;

    			case 3:
    				LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 0);
    				LPC_GPIO1->FIOCLR = (1 << 8);
    				/// SLOW-DOWN-SETTLE
    				/** Reset any previous state parameters */
    				confidence_level_settling = 0;

    				/** In this state do this.. */
    				//u0_dbg_printf("SLOW_SETTLE\n");

    				if (xVal < prev_acc){
    					confidence_level_settled += confidence_level_settled;
    				}else {
    					confidence_level_settled = 0;
    					remove_from_stuck ++;
    				}
    				prev_acc = xVal;

    				/** Condition for state transition */
    				if (confidence_level_settled > definitly_settled || remove_from_stuck > T_remov_from_stuck){
    					acc_at_settled = xVal-3;
    					state = 0;
    				}

    				if (hit_bottom_slow_acc > -50){
    					state = 1;
    				}
    				myEvent = kHalo_Mod_MAE_EV_Stopping;
    				break;

    			}


    			if(previous_state != state){
    				tHalo_Msg* myMsg;
    				myMsg->xnSrc = kHalo_MsgSrc_Mod_MAE;
    				myMsg->xMAE.xnEV = myEvent;

    				gHalo_MHI_BroadCast(pxMA->xpHCtx->xhMHI, myMsg);
    				previous_state = state;


    			}

    		}

    	}

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

size_t gHalo_MAE_Init(tHalo_Ctx* axpHCtx){
	tMotionAnalysis* pxMA = (tMotionAnalysis*)calloc(1, sizeof(tMotionAnalysis));
	if(!pxMA)
	        return NULL;

	initMA(pxMA);
	pxMA->xpHCtx = axpHCtx;
	xTaskCreate(taskReadAS, "rdAS_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);

	return (size_t) pxMA;
}

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
