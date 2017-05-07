/*
 * eint_assignment.c
 *
 *  Created on: Mar 9, 2017
 *      Author: Revthy
 */

#include "LPC17XX.h"
#include "printf_lib.h"
#include "eint_assignment.h"
#include <stdio.h>

#define port0_intr (0x00000001)
#define port2_intr (0x00000004)

typedef struct gpio_intr_list{
	uint32_t irq_bit;
	gpio_cbk_func_t func;
	struct gpio_intr_list* next;
}gpio_interrupt_list_t;

gpio_interrupt_list_t* gpio_port0_list = NULL;
gpio_interrupt_list_t* gpio_port2_list = NULL;


void  gpio_interrupt_register (uint8_t port ,uint8_t pin , interrupt_type_t type , gpio_cbk_func_t funcptr )
{
	gpio_interrupt_list_t* tmp = NULL;
	uint32_t enable_bit = 0x01;

	if (0 == port )
	{
		(rising == type )? (LPC_GPIOINT->IO0IntEnR |= enable_bit << pin) : (LPC_GPIOINT->IO0IntEnF |= enable_bit << pin);
		if( (NULL!= (tmp = malloc(sizeof(gpio_interrupt_list_t)))) && (NULL != funcptr) )
		{
			tmp->func = funcptr;
			tmp->irq_bit = enable_bit << pin;
			tmp->next = gpio_port0_list;
			gpio_port0_list=tmp;
		}
	}
	if (2 == port)
	{
		(rising == type )? (LPC_GPIOINT->IO2IntEnR |= enable_bit << pin) : (LPC_GPIOINT->IO2IntEnF |= enable_bit << pin);
		if( (NULL!= (tmp = malloc(sizeof(gpio_interrupt_list_t)))) && (NULL != funcptr) )
		{
			tmp->func = funcptr;
			tmp->irq_bit = enable_bit << pin;
			tmp->next = gpio_port2_list;
			gpio_port2_list=tmp;
		}
	}
	NVIC_EnableIRQ(EINT3_IRQn);
	u0_dbg_printf("gpio interrupt registered for Port : %d Pin :%d \n", port ,pin);

}

static inline void gpio_interrupt_handler(gpio_interrupt_list_t* port_list , uint32_t rstat ,uint32_t fstat,uint32_t* intclr_reg )
{
	gpio_interrupt_list_t* tmp = port_list ;

	while(tmp != NULL )
	{
		if( tmp->irq_bit & rstat )
		{
			(tmp->func)();
			rstat &= ~(tmp->irq_bit);
		}
		if ( tmp->irq_bit & fstat )
		{
			(tmp->func)();
			fstat &= ~(tmp->irq_bit);
		}
		*(intclr_reg) = tmp->irq_bit;
		tmp = tmp->next;

	}
}

void EINT3_ISR (void)
{
	uint32_t rising_status = 0;
	uint32_t falling_status = 0;

	if(port0_intr == (LPC_GPIOINT->IntStatus & port0_intr))
	{
		rising_status = LPC_GPIOINT->IO0IntStatR;
		falling_status = LPC_GPIOINT->IO0IntStatF;
		gpio_interrupt_handler(gpio_port0_list , rising_status,falling_status, &(LPC_GPIOINT->IO0IntClr));
	}
	if( port2_intr == (LPC_GPIOINT->IntStatus & port2_intr))
	{
		//u0_dbg_printf("Port2 \n");
		rising_status = LPC_GPIOINT->IO2IntStatR;
		falling_status = LPC_GPIOINT->IO2IntStatF;
		gpio_interrupt_handler(gpio_port2_list , rising_status,falling_status, &(LPC_GPIOINT->IO2IntClr));
	}
}

