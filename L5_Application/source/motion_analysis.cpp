/*
 * motion_analysis.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: Abhay and Unni
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
#include "storage.hpp"

#define DELAY_BETWEEN_ACC_READS_TICKS 10 //100ms for SJ One

#define WINDOW_SIZE 100 // Size of window to perform moving average filtering
#define SECOND_WINDOW_SIZE 30 // Size of window to perform moving average filtering the second time
#define CAL_ARR_SIZE 300 // Number of values required to calibrate -> 1 second of data


static int IS_CALIBRATE=1; // Flag to monitor calibration phase.

typedef struct
{
    TaskHandle_t xRdAcc; /// Task handler for MAE
    int16_t sFirstReadings[WINDOW_SIZE]; /// Creating fist smoothening filter
    int16_t sReadings[SECOND_WINDOW_SIZE]; /// Creating second smoothening filter
    int16_t sFirstSmoothenedValue; /// Scalar first smoothened value
    int16_t sSmoothenedValue; /// Scalar second smoothened value
    int16_t sCalReadings[CAL_ARR_SIZE]; /// Creating calibration array
    int16_t sCalibrationOffset=0; /// Calibration value
    tHalo_Ctx* xpHCtx; /// Linker object to glue code
}tMotionAnalysis;


void taskReadAS(void* p)
{


    tMotionAnalysis* pxMA = (tMotionAnalysis*)p;

    int16_t x=0; /// X-axis value read from accelerometer
    int16_t x_after_cal=0; /// X-axis value after calibration offset
    int16_t xVal=0; /// X-axis acceleration value which gets evaluated in the state machine
	static int counter = 0; // To monitor calibration


    /// Initialize moving_average_filters to 0.
    memset(&pxMA->sFirstReadings,0,sizeof(int16_t)*WINDOW_SIZE);
    memset(&pxMA->sReadings,0,sizeof(int16_t)*SECOND_WINDOW_SIZE);

    /// Create event to pass through the glue logic
    eHalo_Mod_MAE_Event myEvent = kHalo_Mod_MAE_EV_Invalid;


    /*
     * De-bounce counter for state
     * transition, post filtering phase.
     */
    uint8_t debounce_counter = 0;
    uint16_t T_definately_state_change = 4;

    /// Monitor post filter monitor state.
    uint8_t postFilterstate = 0;


    volatile uint8_t state = 0; /// Initializing the state to be in stop
    volatile uint8_t previous_state=0; /// Monitor previous state to implement debounce

    while(1)
    {
    	x = AS.getX(); /// Read x-axis accelerometer reading

    	if (IS_CALIBRATE){
    		/// -- Calibration phase --

    		/* *
    		 * Recording N x-axis accelerometer
    		 * readings for calibration.
    		 */
    		pxMA->sCalReadings[counter]=x;


    		counter++;

    		// Indicate calibration
    		LPC_GPIO1->FIOCLR = (1 << 0);


    		if(counter > CAL_ARR_SIZE){
    			/// If calibration array if filled,
    			/// perform calibration.
    			pxMA->sCalibrationOffset = calibration(&(pxMA->sCalReadings[0]),CAL_ARR_SIZE);

    			/// Turn off calibration flag
    			IS_CALIBRATE = 0;

    			/**
    			 * LED indication to monitor
    			 * indicate calibration completion.
    			 */
    			LPC_GPIO1->FIOSET = (1 << 0); // turn off calibration LED

    			LPC_GPIO1->FIOCLR = (1 << 1) | (1 << 4) | (1 << 8);
    			delay_ms(100);

    			LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 8) | (1 << 0);
    		}
    	} else {

    		/// -- Motion analysis phase --

    		x_after_cal = x - pxMA->sCalibrationOffset; /// Compute acceleration after considering calibration offset
    		pxMA->sFirstSmoothenedValue = moving_average_filter(&(pxMA->sFirstReadings[0]),x_after_cal,WINDOW_SIZE); /// First smoothened value
    		pxMA->sSmoothenedValue = moving_average_filter(&(pxMA->sReadings[0]),pxMA->sFirstSmoothenedValue,SECOND_WINDOW_SIZE); /// Second smoothened value

    		xVal = pxMA->sSmoothenedValue;  /// Evaluating on xVal

    		/// Start state machine.
    		switch (state){
    		case 0:
    			///---- STOP ----
    			if (xVal > 20){
    				state = 1;
    			}
    			break;
    		case 1:
    			///---- MOVING ----
    			if (xVal < 10){
    				state = 2;
    			}
    			break;
    		case 2:
    			///---- META STABLE 1 ----
    			if (xVal < -20){
    				state = 3;
    			}
    			if (xVal > 20){
    				state = 1;
    			}
    			break;
    		case 3:
    			///---- SLOWDOWN ----
    			if (xVal > -10){
    				state = 4;
    			}
    			break;
    		case 4:
    			///---- META STABLE ----
    			if (xVal > 20){
    				state = 1;
    			}
    			if (xVal < -10){
    				state = 3;
    			}
    			break;
    		}

#if 0
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

#endif

    		/*
    		 * This acts as a post processing filter to
    		 * eliminate change in states rapidly.
    		 */
    		if(previous_state != state){

    			debounce_counter ++;

    			/// If debounce_counter exceeds limit, change the state
    			if(debounce_counter > T_definately_state_change){

    				postFilterstate = state;
    				debounce_counter = 0; /// Reset debounce counter.


    				/*
    				 * The actual post filtering process
    				 * to update event triggers is done here.
    				 */
    	    		switch(postFilterstate){
    	    		case 0:
    	    			LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 0);
    	    			LPC_GPIO1->FIOCLR = (1 << 8);
    	    			myEvent = kHalo_Mod_MAE_EV_Stopped;
    	    			break;
    	    		case 1:
    	    			LPC_GPIO1->FIOSET = (1 << 0) | (1 << 1) | (1 << 8);
    	    			LPC_GPIO1->FIOCLR = (1 << 4);
    	    			myEvent = kHalo_Mod_MAE_EV_Moving;
    	    			break;
    	    		case 2:
    	    			LPC_GPIO1->FIOSET = (1 << 4) | (1 << 0) | (1 << 8);
    	    			LPC_GPIO1->FIOCLR = (1 << 1);
    	    			myEvent = kHalo_Mod_MAE_EV_Moving;
    	    			break;
    	    		case 3:
    	    			LPC_GPIO1->FIOSET = (1 << 1) | (1 << 4) | (1 << 8);
    	    			LPC_GPIO1->FIOCLR = (1 << 0);
    	    			myEvent = kHalo_Mod_MAE_EV_Stopping;
    	    			break;
    	    		case 4:
    	    			LPC_GPIO1->FIOSET = (1 << 4) | (1 << 0);
    	    			LPC_GPIO1->FIOCLR = (1 << 1) | (1 << 8);
    	    			myEvent = kHalo_Mod_MAE_EV_Stopped;
    	    			break;
    	    		}


    	    		/*
    	    		 * Create message and broadcast
    	    		 */
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
    			}
    		}

    		previous_state = state;

    	}
    	/// Perform this task every 10ms
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
