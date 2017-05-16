/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 */

#include <stdio.h>
#include "utilities.h"
#include <FreeRTOS.h>
#include "task.h"
#include "queue.h"
#include "gpio.hpp"
#include "tasks.hpp"
#include "eint.h"
#include "examples/examples.hpp"

#include "exp_halo_api.h"

#include "LPC17xx.h"

SoftTimer t1(200);
typedef struct {
        SemaphoreHandle_t sem;
        SemaphoreHandle_t sem1;
        QueueHandle_t qh;
}sh_data;
sh_data dat={0};
void isr_func(void)
{
    tHalo_Msg Uio_msg;
    Uio_msg.xnSrc = kHalo_MsgSrc_Mod_UIO;
    Uio_msg.xUIO.xnEV = kHalo_Mod_UIO_EV_Left;
    long yield = 0;
    // give semaphore to the switch_function
    if(xSemaphoreGiveFromISR(dat.sem, &yield))
    {
        t1.reset(200);
        /* Process the interrupt */
        while(!t1.expired());
        LPC_GPIO2->FIOPIN ^= (0x1 << 8); // turning on left side led
        // send this message to the queue.
        xQueueSend(dat.qh, &Uio_msg, 0);
        // yielding interrupt service routine
        portYIELD_FROM_ISR(yield);
    }
}
void isr_func1(void)
{
    tHalo_Msg Uio_msg1;
    Uio_msg1.xnSrc = kHalo_MsgSrc_Mod_UIO;
    Uio_msg1.xUIO.xnEV = kHalo_Mod_UIO_EV_Right;
    long yield = 0;
    // give semaphore to the switch_function
    if(xSemaphoreGiveFromISR(dat.sem1, &yield))
    {
        t1.reset(200);
        /* Process the interrupt */
        while(!t1.expired());
        // yielding interrupt service routine
        LPC_GPIO2->FIOPIN ^= (0x1 << 9);

        // send this message to the queue
        xQueueSend(dat.qh, &Uio_msg1, 0);
        portYIELD_FROM_ISR(yield);
    }
}
class halo_led:public scheduler_task
{
        size_t xhUIO;
    public:
        halo_led(uint8_t priority, size_t xUio):scheduler_task("led_task", 512, priority), xhUIO(xUio)
    {
            // initialise led and switch gpio pin
            LPC_GPIO2->FIODIR |= (0x1 << 8);
            LPC_GPIO2->FIODIR |= (0x1 << 9);

            // switch in on port 0 pin 0 and pin 1
            LPC_GPIO0->FIODIR &= ~(0x1);
            LPC_GPIO0->FIODIR &= ~(0x1 << 1);

            // turning off the leds initially
            LPC_GPIO2->FIOPIN |= (0x1 << 8);
            LPC_GPIO2->FIOPIN |= (0x1 << 9);



    }

        bool init(void)
        {
            dat.qh = xQueueCreate(1, sizeof(tHalo_Msg));

            dat.sem = xSemaphoreCreateBinary();
            dat.sem1 = xSemaphoreCreateBinary();
            // eint interrupt source
            eint3_enable_port0(0, eint_rising_edge, isr_func);
            eint3_enable_port0(1, eint_rising_edge, isr_func1);
            return true;
        }

        bool run(void* p)
        {
            // receive data from queue;
            // send this message to others in the queue;
            // wait for the message receive from the switch isr
            tHalo_Msg Uio_msg;

            // get the Uio_msg from isr
            xQueueReceive(dat.qh, &Uio_msg, portMAX_DELAY);

            // send the response to the Message Handler
            gHalo_MHI_BroadCast(xhUIO, &Uio_msg);

            return true;
        }


};
/** @{ UIO API */
size_t gHalo_UIO_Init(tHalo_Ctx* axpHCtx)
{
    scheduler_add_task(new halo_led(PRIORITY_MEDIUM, axpHCtx->xhUIO));

    return 1;
}
/** @} UIO API */
