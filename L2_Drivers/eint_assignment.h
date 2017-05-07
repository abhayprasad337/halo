/*
 * eint_assignment.h
 *
 *  Created on: Mar 10, 2017
 *      Author: Revthy
 */

#ifndef L2_DRIVERS_EINT_ASSIGNMENT_H_
#define L2_DRIVERS_EINT_ASSIGNMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	rising =0,
	falling =1
}interrupt_type_t;

typedef void (*gpio_cbk_func_t) (void);
void EINT3_Init(void);
void  gpio_interrupt_register (uint8_t port ,uint8_t pin , interrupt_type_t type , gpio_cbk_func_t funcptr );

#ifdef __cplusplus
}
#endif
#endif /* L2_DRIVERS_EINT_ASSIGNMENT_H_ */
