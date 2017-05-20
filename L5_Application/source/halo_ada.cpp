/*
 * halo_ada.cpp
 * AUTHOR: ABHAY PRASAD
 *
 * The ADA(Alert Display Array) module is used to
 * control the board on the helmet (Board 2).
 *
 * This unit monitors the messages obtained from
 * board 1 and performs actions accordingly.
 *
 * Functions of this board:
 * 1. To blink indicators - left or right
 *
 * 2. To display moving or slowing down on helmet
 *     - The frequency of blinking changes on the fly.
 */


/**
 * Connections to external components
 *
 * P1.28 - Buzzer
 * P1.29 - Left indicator
 * P1.30 - Right indicator
 *
 * P2.6 - Ultrasound buzzer
 *
 */

#include "debug_e.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart0_min.h"
#include "motion_analysis.hpp"
#include "printf_lib.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "utilities.h"
#include "halo_ada.hpp"
#include "exp_halo_api.h"

#include "LPC17xx.h"
#include "lpc_sys.h"
#include "soft_timer.hpp"
#include "lpc_pwm.hpp"

/*
 * Hardware configuration of
 * external LED interface to the helmet
 * for Board 2
 *
 * LED connections
 * P0.29 - LEFT
 * P0.30 - LEFT
 *
 * P0.0 - RIGHT
 * P0.1 - RIGHT
 *
 * P2.6 - MOVING STATE
 * P2.7 - MOVING STATE
 *
 */


#define LEFT_LED_CONTROL_GPIO1    29
#define LEFT_LED_CONTROL_GPIO2    30

#define RIGHT_LED_CONTROL_GPIO1   0
#define RIGHT_LED_CONTROL_GPIO2   1

#define CENTRAL_LED_CONTROL_GPIO1 6
#define CENTRAL_LED_CONTROL_GPIO2 7



#define BLINK_MOVING 100
#define BLINK_SLOW_DOWN 500
#define BLINK_STOP 1


static bool IS_LEFT_SET =  0;
static bool IS_RIGHT_SET =  0;

static uint16_t blinkSpeed=BLINK_STOP;

inline void trigger_moving(){
	blinkSpeed = BLINK_MOVING;
}

inline void trigger_slow_down(){
	blinkSpeed = BLINK_SLOW_DOWN;
}

inline void trigger_stopped(){
	blinkSpeed = BLINK_STOP;
}


PWM pwm2(PWM::pwm2, 50);

void buzz_warning(){
	pwm2.set(0.5);
	delay_ms(50);
	pwm2.set(0);
}

void buzz_critical(){
	pwm2.set(90);
	delay_ms(1000);
	pwm2.set(0);
}


/**
 * Soft timer to blink indicator for
 * x time ticks
 */
SoftTimer St_blinkIndicator(1000);

typedef struct{
  tHalo_Ctx * xpCtx;
  uint32_t indicators;
  uint16_t blinkSpeed;
}adaUnit_t;


void trigger_left(){
	St_blinkIndicator.reset();
	IS_RIGHT_SET = false;
	IS_LEFT_SET = true;
}

void trigger_right(){
	St_blinkIndicator.reset();
	IS_LEFT_SET = false;
	IS_RIGHT_SET = true;
}



/*
 * Basic framework of function to blink
 * LEDs for indicators
 */
void blinkMe(uint32_t blinkThisPin){
	LPC_GPIO0->FIOCLR = blinkThisPin;
	delay_ms(50);
	LPC_GPIO0->FIOSET = blinkThisPin;
	delay_ms(50);
}

/**
 * ABSTRACT:
 * 	This task is used to control the blinking of the
 * 	indicators on the helmet according to the button press
 *
 *  The indicators are set to blink for n(5) seconds. A
 *  soft timer is used to monitor this.
 *
 */
void doBlinkIndicator(void *p){
	uint32_t indicators = 0;

	while(1){
		/**
		 * Turn off indicators after n seconds.
		 */
		if(St_blinkIndicator.expired()){
			IS_RIGHT_SET = false;
			IS_LEFT_SET = false;
		}

		indicators = (IS_LEFT_SET << LEFT_LED_CONTROL_GPIO1)  | (IS_LEFT_SET << LEFT_LED_CONTROL_GPIO2)  | (IS_RIGHT_SET << RIGHT_LED_CONTROL_GPIO1) | (IS_RIGHT_SET << RIGHT_LED_CONTROL_GPIO2);
		blinkMe(indicators);
	}
}


void cautionLights(uint8_t blinkInThisSpeed){
	LPC_GPIO2->FIOCLR = 1 << CENTRAL_LED_CONTROL_GPIO1;
    LPC_GPIO2->FIOCLR = 1 << CENTRAL_LED_CONTROL_GPIO2;
	delay_ms(blinkInThisSpeed);
	LPC_GPIO2->FIOSET = 1 << CENTRAL_LED_CONTROL_GPIO1;
	LPC_GPIO2->FIOSET = 1 << CENTRAL_LED_CONTROL_GPIO2;
	delay_ms(blinkInThisSpeed);
}




/**
 * This task is used to control the caution
 * LEDS that are behind the helmet.
 *
 * The caution LEDs blinks according to the state
 * of motion of the cyclist.
 *
 * 1. In moving - blink every 100 ms
 * 2. In slowing down - blink 5000 ms
 * 3. In stop - blink 1000ms 1000 ms
 */
void doBlinkMotion(void *p){
	while(1){
		cautionLights(blinkSpeed);
	}
}


/**
 * Initialize ADA unit.
 */
size_t gHalo_ADA_Init(tHalo_Ctx* axpHCtx){
	adaUnit_t * pxADA = (adaUnit_t*) calloc (1,sizeof(adaUnit_t));
	if (!pxADA)
		return (size_t)NULL;

	pxADA->xpCtx = axpHCtx;

	LPC_GPIO1->FIODIR |= (1 << LEFT_LED_CONTROL_GPIO1)  | (1 << LEFT_LED_CONTROL_GPIO2)  | (1 << CENTRAL_LED_CONTROL_GPIO1) |(1 << CENTRAL_LED_CONTROL_GPIO2) | (1 << RIGHT_LED_CONTROL_GPIO1) | (1 << RIGHT_LED_CONTROL_GPIO2);

	// Create tasks here
	/*
	 * Two tasks, one for indicator and other for displaying
	 * the motion of the cyclist.
	 */
	xTaskCreate(doBlinkIndicator,"ADA_CONTROL",512,NULL,1,NULL);
	xTaskCreate(doBlinkMotion,"ADA_MOT_CONTROL",512,NULL,1,NULL);


	return (size_t) pxADA;
}


bool gHalo_ADA_Show(size_t axhADA, tHalo_Msg* axpMsg){
	switch(axpMsg->xnSrc){
		case (kHalo_MsgSrc_Mod_USS):
		/** < from Ultra Sound */
				switch(axpMsg->xUSS.xnWL){
					case(kHalo_Mod_USS_WL_Warning):
						buzz_warning();
						break;
					case(kHalo_Mod_USS_WL_Critical):
						buzz_critical();
						break;
					case(kHalo_Mod_USS_WL_Safe):
					    /** do nothing */
					    break;
					default:
					    LOGE("unhandled msg at ADA; from USS\n");
					    break;
				}
				break;
		case(kHalo_MsgSrc_Mod_MAE):
		/**< from MAE */
				switch(axpMsg->xMAE.xnEV){
				case (kHalo_Mod_MAE_EV_Moving):
						trigger_moving();
						break;
				case (kHalo_Mod_MAE_EV_Stopping):
						trigger_slow_down();
						break;
				case(kHalo_Mod_MAE_EV_Stopped):
						trigger_stopped();
						break;
				default:
				        LOGE("unhandled msg at ADA; from MAE\n");
				    break;
				}
				break;
		case(kHalo_MsgSrc_Mod_UIO):
		/**< from UIO */
			switch(axpMsg->xUIO.xnEV){
				case (kHalo_Mod_UIO_EV_Left):
					trigger_left();
					break;
				case(kHalo_Mod_UIO_EV_Right):
					trigger_right();
					break;
				default:
				    LOGE("unhandled msg at ADA; from UIO\n");
				    break;
			}
		break;
	}
	return NULL;
}

