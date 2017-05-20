/*
 * motion_analysis.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: unnik
 */


#define VERBOSE_E
#include "debug_e.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart0_min.h"
#include "motion_analysis.hpp"
#include "io.hpp"
#include "printf_lib.h"
#include "stdlib.h"
#include "stdio.h"
#include "soft_timer.hpp"
#include "string.h"
#include "utilities.h"
#include "exp_halo_api.h"

#define DELAY_BETWEEN_ACC_READS_TICKS 10 //100ms for SJ One

/**
 * tick is RTOS System Tick Interrupt
 * configTICK_RATE_HZ; //the number of ticks in 1 second
 * portTICK_PERIOD_MS; //each tick shall happen in these many ms
 * So our System Tick Interrupt is 1ms configTICK_RATE_HZ=1000 (which is 1 tick in 1ms)
 */

#define WINDOW_SIZE 100 // Size of window to perform moving average filtering
#define SECOND_WINDOW_SIZE 30 // Size of window to perform moving average filtering the second time
#define CAL_ARR_SIZE 300 // Number of values required to calibrate -> 1 second of data


static int IS_CALIBRATE=1;

typedef struct
{
    TaskHandle_t xRdAcc;
    int16_t sFirstReadings[WINDOW_SIZE];
    int16_t sReadings[SECOND_WINDOW_SIZE];
    int16_t sFirstSmoothenedValue;
    int16_t sSmoothenedValue;
    int16_t sCalReadings[CAL_ARR_SIZE];
    int16_t sCalibrationOffset=0;
    tHalo_Ctx* xpHCtx;

}tMotionAnalysis;


void taskReadAS(void* p)
{
    tMotionAnalysis* pxMA = (tMotionAnalysis*)p;
    int16_t x=0;
    int16_t x_after_cal=0;
	static int counter = 0;
    int16_t xVal;

    // Initialize moving_average_filter readings to 0.
    memset(&pxMA->sFirstReadings,0,sizeof(int16_t)*WINDOW_SIZE);
    memset(&pxMA->sReadings,0,sizeof(int16_t)*SECOND_WINDOW_SIZE);




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
    int16_t T_isSlowDown = -10;
    int16_t T_remov_from_stuck = 10;
    int16_t T_ismoving_from_stop = 40;

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
    int16_t definitly_settled = 3;
    int16_t definitely_out_of_stop = 2;

    eHalo_Mod_MAE_Event myEvent = kHalo_Mod_MAE_EV_Invalid;


    volatile uint16_t inMotioncnt = 0;
    volatile uint16_t inStopcnt = 0;

    /*
     * De-bounce counter for state
     * transition, post filtering phase.
     */
    uint8_t debounce_counter = 0;
    uint16_t T_definately_state_change = 0;

    uint8_t postFilterstate = 0;

    while(1)
    {
    	x = AS.getX();
    	if (IS_CALIBRATE){
    		/// -- Calibration phase --

    		/* Record N values and take the average of them */
    		//pxMA->sFirstSmoothenedValue = moving_average_filter(&(pxMA->sFirstReadings[0]),x,WINDOW_SIZE);
    		pxMA->sCalReadings[counter]=x;


    		counter++;
    		LPC_GPIO1->FIOCLR = (1 << 0); // Calibration LED is ON

    		if(counter > CAL_ARR_SIZE){
    			pxMA->sCalibrationOffset = calibration(&(pxMA->sCalReadings[0]),CAL_ARR_SIZE);
    			acc_at_settled = pxMA->sCalibrationOffset;
    			LPC_GPIO1->FIOSET = (1 << 0); // turn off calibration LED
    			IS_CALIBRATE = 0;
    			LPC_GPIO1->FIOCLR = (1 << 1) | (1 << 4) | (1 << 8);
    			delay_ms(500);
    			LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 8) | (1 << 0);
    		}
    	} else {

    		/// -- Motion analysis phase --

    		x_after_cal = x - pxMA->sCalibrationOffset;
    		pxMA->sFirstSmoothenedValue = moving_average_filter(&(pxMA->sFirstReadings[0]),x_after_cal,WINDOW_SIZE);
    		pxMA->sSmoothenedValue = moving_average_filter(&(pxMA->sReadings[0]),pxMA->sFirstSmoothenedValue,SECOND_WINDOW_SIZE);


    		/// Perform motion and slow down detection.
    		/// This has to be changed to pxMA->sSmoothenedValue.
    		/// This is because the algorithm thresholds have been set for this type of configuration.
    		xVal = pxMA->sSmoothenedValue;

    		switch(state){
    		case 0:
    			if (xVal < -15){
    				inMotioncnt ++;
    			} else {
    				inMotioncnt = 0;
    			}


    			if (inMotioncnt > 5){
    				inStopcnt =0;
    				state = 1;
    			}

    			break;
    		case 1:
    			if (xVal > -5){
    				inStopcnt++;
    			}


    			if (inStopcnt > 5){
    				inMotioncnt = 0;
    				state = 0;
    			}

    			break;
    		}

#if 0
    		switch(state){
    		case 0: // Stop or Slow down
    			if (xVal > 10){
    				inMotioncnt ++;
    			} else {
    				inMotioncnt = 0;
    			}


    			if (inMotioncnt > 3){
    				inMotioncnt=0;
    				state = 1;
    			}
    			break;

    		case 2: // Motion
    			if (xVal < -10){
    				inStopcnt++;
    			} else {
    				inStopcnt=0;
    			}

    			if(inStopcnt > 3){
    				inStopcnt=0;
    				state = 0;
    			}
    			break;
    		}

#endif




#if 0
    		switch(state){

    		case 0:

    			/// STOP
    			/** Reset any previous state parameters */
    			confidence_level_settled = 0;
    			remove_from_stuck = 0;

    			/** In this state do this.. */
    			//u0_dbg_printf("STOP\n");

    			if((xVal < acc_at_settled+10) && (xVal > acc_at_settled-10)){
    				confidence_level_moving = 0;
    			} else {
    				confidence_level_moving = confidence_level_moving + 1;
    			}
    			prev_acc = xVal;


    			/** Condition for state transition */
/*    			if (confidence_level_moving > definitely_out_of_stop){
    				if  (xVal > T_ismoving_from_stop){
    					state = 1;
    				}else{
    					if (xVal < T_isSlowDown){
    						state = 2;
    					}
    				}
    			}*/

    			if (confidence_level_moving>definitely_out_of_stop){
    				if( xVal > acc_at_settled+10){
    					state = 1;
    				} else {
    					if (xVal < T_isSlowDown){
    						state = 2;
    					}
    				}
    			}


    			myEvent = kHalo_Mod_MAE_EV_Stopped;
    			break;
    		case 1:

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
    			myEvent = kHalo_Mod_MAE_EV_Stopping; // Settling is considered as a stop itself
    			break;

    		} // << state

#endif


    		if(previous_state != state){

    			debounce_counter ++;
    			if(debounce_counter > T_definately_state_change){
    				tHalo_Msg myMsg;
    				myMsg.xnSrc = kHalo_MsgSrc_Mod_MAE;
    				myMsg.xMAE.xnEV = myEvent;
    				LOGD("DEBUGME\n");

    				if(!gHalo_MHI_BroadCast(pxMA->xpHCtx->xhMHI, &myMsg))
    				{
    					LOGE("BCAST ERR!\n");
    				}
    				else
    				{
    					LOGV("BCAST SUCCESS\n");
    				}

    				postFilterstate = state;
    			}
    		}
    		previous_state = state;

    		switch(postFilterstate){
    		case 0:
    			LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 0);
    			LPC_GPIO1->FIOCLR = (1 << 8);
    			break;
    		case 1:
    			LPC_GPIO1->FIOSET = (1 << 0) | (1 << 1) | (1 << 8);
    			LPC_GPIO1->FIOCLR = (1 << 4);
    			break;
    		case 2:
    			LPC_GPIO1->FIOSET = (1 << 4) | (1 << 0) | (1 << 8);
    			LPC_GPIO1->FIOCLR = (1 << 1);
    			break;
    		case 3:
    			LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 8);
    			LPC_GPIO1->FIOCLR = (1 << 0);
    			break;
    		}


    	}
    	vTaskDelay(DELAY_BETWEEN_ACC_READS_TICKS);
    }
}


/**
 * ABSTRACT: This is used to trigger the motion analysis engine
 * 			 using the glue logic.
 */
size_t gHalo_MAE_Init(tHalo_Ctx* axpHCtx){

	/** Create motion analysis object */
	tMotionAnalysis* pxMA = (tMotionAnalysis*)calloc(1, sizeof(tMotionAnalysis));
	if(!pxMA)
	        return (size_t)NULL;

	/** Object to share resources with glue logic */
	pxMA->xpHCtx = axpHCtx;

	/** Create and start motion analysis engine task */
	xTaskCreate(taskReadAS, "rdAS_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);

	return (size_t) pxMA;
}


/**
 * ABSTRACT: This function is to trigger the motion analysis engine.
 */
void* xStartMotionAnalysis()
{
    /** Create Motion Analysis object */
	tMotionAnalysis* pxMA = (tMotionAnalysis*)calloc(1, sizeof(tMotionAnalysis));
    if(!pxMA)
        return NULL;

    /** Create and start motion analysis engine task */
    xTaskCreate(taskReadAS, "rdAS_task", 512 * 4, pxMA, PRIORITY_MEDIUM, &pxMA->xRdAcc);

    return (void*)pxMA;
}
