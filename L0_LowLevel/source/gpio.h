/*
 * gpio.h
 *
 *  Created on: Feb 13, 2017
 *      Author: Revthy
 */


#ifndef L0_LOWLEVEL_GPIO_H_
#define L0_LOWLEVEL_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include  "LPC17XX.h"


/*Bit SET macro*/
#define SET(x)          (1 << x)
#define CLR(x)          (~(1<<x))
#define SET_2BITS(n,pin)  (n << (pin*2))
#define CLR_2BITS(x)    (~(3 << (x*2)))
#define SET_2BIT_POS(n,pos) (n << pos)
#define CLR_2BIT_POS(pos)  (~(3 << pos))


#define REG_L   15u
#define REG_H   31u

/*
 * Pin Configuration Enumerations
 * Reference: LPC74XX datasheet
 */

typedef enum
{
	gpio =0,
	first_alternate_func=1,
	second_alternate_func=2,
	third_alternate_func=3
}pin_func_t;

typedef enum
{
	input =0,
	output=1
}pin_mode_t;

typedef enum
{
	pullup =0,
	repeater=1,
	none=2,
	pulldown=3
}pin_input_mode_t;

typedef enum
{
	pushpull =0,
	opendrain=1
}pin_output_mode_t;


/*********Function Declaration***********/
/*
 * Function Interfaces to set pin configurations
 * these functions can be called to configure and use any gpio port
 */

extern void configurePortPin(uint8_t portnum,
		              uint8_t pin,
					  pin_func_t func,
					  pin_mode_t mode,
					  pin_input_mode_t imode,
					  pin_output_mode_t omode);
//To do
void GPIO_IRQ_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* L0_LOWLEVEL_GPIO_H_ */
