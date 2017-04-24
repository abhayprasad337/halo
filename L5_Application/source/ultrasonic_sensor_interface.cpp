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

/* Macro declration*/
#define TRIGGER_CYCLE pdMS_TO_TICKS(100)
#define CALCULATE_DISTANCE(duration)  ((duration/2000)*330);


/*C functions*/
extern "C"
{
   void echo_irq_callback(void)
   {
	   long yieldRequired = 0;
       if(xSemaphoreGiveFromISR(scheduler_task::getSharedObject(shared_ultrasonicEchoSemaphore),&yieldRequired) !=0 )
       u0_dbg_printf("echo_irq_callback - Semaphore Give \n ");
	   //portYIELD_FROM_ISR(yieldRequired);
   }

}

/*Global function declaration*/
void PeriodicTriggerCallback(void*);

/*Member function definition*/
US_PeriodicTrigger::US_PeriodicTrigger(uint8_t priority) : scheduler_task("usTrigger",512*2,priority),
		                                                   mxSemaphoreTriggerSignal(NULL),
														   mstartTrigger(false)
{
	mxTimerPeriodicTrigger.reset(100);
}

bool US_PeriodicTrigger::init(void)
{
	//u0_dbg_printf("\n US_PeriodicTrigger init \n");

	configurePortPin(2,3,gpio,output,pulldown,pushpull);              //Configure P2.3 as output in push-pull mode
	LPC_GPIO2->FIOCLR= (1 << 3);

#if 0
	mxTimerPeriodicTrigger = xTimerCreate("us_trigger", TRIGGER_CYCLE , pdTRUE , NULL , PeriodicTriggerCallback);
    if(NULL != mxTimerPeriodicTrigger)
    {
    	mstartTrigger = xTimerStart(mxTimerPeriodicTrigger,40);
    }
#endif

	mxSemaphoreTriggerSignal = xSemaphoreCreateBinary();
	if(NULL!= mxSemaphoreTriggerSignal)
	{
		addSharedObject(shared_ultrasonicTrigSemaphore,mxSemaphoreTriggerSignal);
	}
	return (NULL != mxSemaphoreTriggerSignal);
}


bool US_PeriodicTrigger::run(void* p)
{
	if(mxTimerPeriodicTrigger.expired())
	{
		//u0_dbg_printf("\n US_PeriodicTrigger expired\n");
		PeriodicTriggerCallback(NULL);
		mxTimerPeriodicTrigger.restart();
		//mstartTrigger = xTimerStart(mxTimerPeriodicTrigger,40);
	}

	return true;
}


void PeriodicTriggerCallback(void* p)
{
	//u0_dbg_printf("\n US_PeriodicTrigger Callback \n");
	LPC_GPIO2->FIOSET= (1 << 3);
	delay_ms(10);
	LPC_GPIO2->FIOCLR= (1 << 3);
	xSemaphoreGive(scheduler_task::getSharedObject(shared_ultrasonicTrigSemaphore));
}



US_EchoDetect::US_EchoDetect(uint8_t priority) : scheduler_task("usEchoDetect",512*3,priority),mEchoDuration(0),
		                                         mstartTime(0),
												 mendTime(0),
												 mdistance(0),
		                                         mxSemaphoreWarningL1(NULL),
		                                         mxSemaphoreTriggeredSignal(NULL),
		                                         mxSemaphoreEchoRcvdSignal(NULL),
												 maliveEchoDetect(NULL)
{
}

bool US_EchoDetect::init(void)
{
	u0_dbg_printf("\n US_EchoDetect init \n");
	gpio_interrupt_register (2 ,4 , rising ,echo_irq_callback );      //Configure P2.4 for Raising edge interrupt

	mxSemaphoreWarningL1 = xSemaphoreCreateBinary();
	maliveEchoDetect = xEventGroupCreate();
	mxSemaphoreEchoRcvdSignal = xSemaphoreCreateBinary();

	if(NULL!= mxSemaphoreWarningL1)
	{
		addSharedObject(shared_ultrasonicWarnSemaphore,mxSemaphoreWarningL1);
	}

	if(NULL!= mxSemaphoreEchoRcvdSignal)
	{
			addSharedObject(shared_ultrasonicEchoSemaphore,mxSemaphoreEchoRcvdSignal);
	}

	return ((NULL!= mxSemaphoreEchoRcvdSignal) && (NULL != mxSemaphoreWarningL1) &&(NULL !=maliveEchoDetect));

}

bool US_EchoDetect::run(void* p)
{
	//u0_dbg_printf("\n US_EchoDetect run \n");

	if(xSemaphoreTake(getSharedObject(shared_ultrasonicTrigSemaphore),portMAX_DELAY) !=0 )
	{
	   u0_dbg_printf("shared_ultrasonicTrigSemaphore - Semaphore Taken \n ");
	   mstartTime = sys_get_uptime_ms();
	}

	if(xSemaphoreTake(mxSemaphoreEchoRcvdSignal,portMAX_DELAY) !=0 )
	{
		mendTime = sys_get_uptime_ms();
		mEchoDuration = mendTime - mstartTime;
		mdistance=CALCULATE_DISTANCE(mEchoDuration)
		u0_dbg_printf("shared_ultrasonicEchoSemaphore - Semaphore Taken \n Duration calculated = %d \n Distance = %f \n ",mEchoDuration,mdistance);
	}
	return 1;

}


