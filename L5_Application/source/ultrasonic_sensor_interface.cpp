/*
 *UltrasonicSensorInterface.cpp
 *
 *@owner : Revathy
 *@created : 22 Apr 2017
 *@file : Implements the Ultrasonic sensor interface module
 *@version : 1
 *
 * Task handling UltraSonic Sensor Module(HC-SR04)
 * @module operation : UltraSonic sensor operates with a pulse of 10 microsecond triggered by the Microcontroller gpio port pin.
 *                     The sensor sends out 8 chirps and detects the echo pulse
 *                     The echo output pin raising edge is detected by microcontroller input pin and the time between start of trigger
 *                     and echo received is calculated
 *
 * @formula          : Distance = (Speed of sound) * (Time between Trigger and Echo /2)
 *                              = (330 m/s)*( T/2 s)
 *
 * @threshold        : If the distance is less than 1m, Turn on indicators
 */
#include <stdio.h>
#include "ultrasonic_sensor_interface.hpp"
#include "printf_lib.h"
#include "shared_handles.h"
#include "L0_LowLevel/source/gpio.h"
#include "L2_Drivers/eint_assignment.h"
#include "utilities.h"
#include "lpc_sys.h"
#include "L2_Drivers/eint.h"

/* Macro declration*/
#define TRIGGER_CYCLE                 200                                   /*Trigger cycle for USS */
#define CALCULATE_DISTANCE(duration)  (((float)duration*(float)(0.034/2)))  /* 340 m/s , so 0.034 cm per us*/

static uint32_t pulseEndTime=0;
static uint32_t pulseStartTime=0;

/*C EINT ISR functions*/
extern "C"
{
   void echo_raising_irq_callback(void)
   {
	   u0_dbg_printf("\n 1.echo_raising_irq_callback \n ");
	   pulseStartTime=(unsigned int)sys_get_uptime_us();
   }

   void echo_falling_irq_callback(void)
     {
  	   pulseEndTime=(unsigned int)sys_get_uptime_us();
  	   u0_dbg_printf("\n 2.Send end time to Queue \n ");
  	   if(NULL != scheduler_task::getSharedObject(shared_ultrasonicTimerQueue))
  	   {
  		   xQueueSendFromISR(scheduler_task::getSharedObject(shared_ultrasonicTimerQueue),&pulseEndTime,NULL);
  	   }

  	 }

}


/*Member function definition*/
USS_PeriodicTriggerTask::USS_PeriodicTriggerTask(uint8_t priority,size_t pMsgHandler) : scheduler_task("USSTrigger",512*8,priority),MsgHandler(pMsgHandler),
                                                                   mstartTime(0),
																   mendTime(0),
																   mstatePrev(kHalo_Mod_USS_WL_Safe),
																   mstateCurrent(kHalo_Mod_USS_WL_Safe),
																   mEchoDuration(0),
																   mdistance(0),
																   mxUSSCmdQ(NULL)
{
	mxTimerPeriodicTrigger.reset(TRIGGER_CYCLE);
}

USS_PeriodicTriggerTask::USS_PeriodicTriggerTask(uint8_t priority):scheduler_task("USS_Default_Trigger",512*8,priority),MsgHandler(0),
		                                                                   mstartTime(0),
																		   mendTime(0),
																		   mstatePrev(kHalo_Mod_USS_WL_Safe),
																		   mstateCurrent(kHalo_Mod_USS_WL_Safe),
																		   mEchoDuration(0),
																		   mdistance(0),
																		   mxUSSCmdQ(NULL)
{
	mxTimerPeriodicTrigger.reset(TRIGGER_CYCLE);
}

bool USS_PeriodicTriggerTask::init(void)
{
	u0_dbg_printf("\n US_PeriodicTrigger init \n");

	/*Create Queue to receive command from EINT3 ISR to calculate Echo pulse width*/
	mxUSSCmdQ = xQueueCreate(1,sizeof(uint32_t));
	if(NULL!= mxUSSCmdQ)
	{
		addSharedObject(shared_ultrasonicTimerQueue,mxUSSCmdQ);
	}

	/*Configure P2.3 as output to send Trigger pulse to USS Trigger pin*/
	configurePortPin(2,3,gpio,output,pulldown,pushpull);

	LPC_GPIO2->FIOCLR= SET_2BIT_POS(7,3);
	//LPC_GPIO2->FIOCLR= (1 << 3);
	//LPC_GPIO2->FIOCLR= (1 << 4);
	//LPC_GPIO2->FIOCLR= (1 << 5);

	/*Register P2.4 & P2.5 for EINT3 Interrupt for rising and falling edge to measure echo pulse width */
	eint3_enable_port2(5,eint_rising_edge,echo_raising_irq_callback);
	eint3_enable_port2(4,eint_falling_edge,echo_falling_irq_callback);

	return (NULL != mxUSSCmdQ);
}


bool USS_PeriodicTriggerTask::run(void* p)
{
	tHalo_Msg US_Msg = {kHalo_MsgSrc_Mod_USS,kHalo_Mod_USS_WL_Safe};

	if(mxTimerPeriodicTrigger.expired())
	{
		u0_dbg_printf("\n 0.US_PeriodicTriggerTimer expired\n");

		LPC_GPIO2->FIOSET= (1 << 3);
		delay_us(10);
		LPC_GPIO2->FIOCLR= (1 << 3);

		if(xQueueReceive(mxUSSCmdQ,&(USS_PeriodicTriggerTask::mendTime),portMAX_DELAY))
		{
        	u0_dbg_printf("\n 4.Received start & End time ");
			mEchoDuration = pulseEndTime - pulseStartTime;
		    mdistance= CALCULATE_DISTANCE(mEchoDuration);
		    runStateMachine();
		    US_Msg.xUSS.xnWL = mstateCurrent ;

		    if(gHalo_MHI_BroadCast(MsgHandler, &US_Msg))
		    {
		    	u0_dbg_printf("\n 5.gHalo_MHI_BroadCast ");
		    }
		}

		u0_dbg_printf("\n 6.Duration calculated = %ld "
						"\n Distance = %f \n start=%ld \n end=%ld \n ",mEchoDuration,mdistance,pulseStartTime,pulseEndTime);

		mxTimerPeriodicTrigger.restart();
	}

	return true;
}

void USS_PeriodicTriggerTask::runStateMachine(void)
{
	mstatePrev= mstateCurrent;

	if( mdistance < 200 )
	{
		mstateCurrent = kHalo_Mod_USS_WL_Critical;

		//Send it to Task
		u0_dbg_printf("\n Critical !");

	}
	else if( mdistance >= 200 && mdistance <= 400 )
	{
		mstateCurrent = kHalo_Mod_USS_WL_Warning;

		u0_dbg_printf("\n warning !");
	}
	else if (mdistance > 400 )
	{
		mstateCurrent = kHalo_Mod_USS_WL_Safe;
		u0_dbg_printf("\n safe !");

	}
}

size_t gHalo_USS_Init(tHalo_Ctx* axpHCtx)
{
	USS_PeriodicTriggerTask* USS_Handler = new USS_PeriodicTriggerTask(PRIORITY_HIGH, axpHCtx->xhMHI );
	scheduler_add_task(USS_Handler);
	return (size_t)USS_Handler;
}


