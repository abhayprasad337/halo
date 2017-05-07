/*
 * eint3_monitoring_hw_int.cpp
 *
 * @brief This task handles EINT3 interrupt on GPIO ports Port0 and Port2
 *
 *  Created on: Mar 13, 2017
 *      Author: unnik
 */

#include "eint3_monitoring_hw_int.hpp"
#include "LPC17xx.h"
#include "printf_lib.h"
#include "string.h"

#define RELEASE_MY_CODE
#ifdef RELEASE_MY_CODE
#define LOGD(...)
#else
#define LOGD(...) u0_dbg_printf(__VA_ARGS__)
#endif

#define eint3_monitor_PORT0_IDX 0
#define eint3_monitor_PORT2_IDX 1

QueueHandle_t xQueueEINT3;

#ifdef __cplusplus
extern "C"
{

/**
 * Ways to register the ISR():
 * 1) we could change the g_pfnVectors[EINT3] to EINT3_IRQHandlerTest
 * instead of isr_forwarder_routine()
 * But, as this is a Test function and later to make sense and not to cause trouble,
 * am using isr_register() function
 * 2) we could go ahead and use the same name EINT3_IRQHandler() so that this definition is used by compiler
 * and not the weak definition
 * But this is not possible as non-weak EINT3_IRQHandler() is defined in eint3.c
 * 3) user isr_register(); but yes there will be a slight overhead. (this overhead is same as with 2) anyway )
 * */
void EINT3_IRQHandlerTest(void);
void EINT3_IRQHandlerTest(void)
{
    /** We support only kINT_EdgeTrig_R here */
    //read Status Register and check if the INT is for rising edge
    uint32_t nIO0StatR = LPC_GPIOINT->IO0IntStatR;
    uint32_t nIO2StatR = LPC_GPIOINT->IO2IntStatR;
    uint32_t i;
    tPINInfo pinInfo = {0};
    LOGD("EINT3 received\n");
    for(i = 0; i < 32; i++)
    {
        //eint3_monitor_PORT0; R
        if(nIO0StatR & (1 << i)) //INT is active
        {
            LOGD("EINT3 received at P0.%d\n", i);
            pinInfo.nAssociatedINTTypes = kINT_EdgeTrig_R;
            pinInfo.nPort = eint3_monitor_PORT0;
            pinInfo.nPIN = i;
            pinInfo.ullTS = SoftTimer::getCurrentTimeMs();
            /*SendFromISR can fail if Q is full; currently we dont take care of the failure condition
             * and its documented in the eint3_monitoring_hw_int.hpp API header */
            xQueueSendFromISR(xQueueEINT3,
                    &pinInfo,
                    NULL /**< BaseType_t *pxHigherPriorityTaskWoken is optional from FreeRTOS V7.3.0, and we use in SJOne, V8.1.2 */);
            // clear the INT
            LPC_GPIOINT->IO0IntClr |= (1 << i);
        }
        //eint3_monitor_PORT2; R
        if(nIO2StatR & (1 << i)) //INT is active
        {
            LOGD("EINT3 received at P2.%d\n", i);
            pinInfo.nAssociatedINTTypes = kINT_EdgeTrig_R;
            pinInfo.nPort = eint3_monitor_PORT2;
            pinInfo.nPIN = i;
            pinInfo.ullTS = SoftTimer::getCurrentTimeMs();
            xQueueSendFromISR(xQueueEINT3,
                    &pinInfo,
                    NULL /**< BaseType_t *pxHigherPriorityTaskWoken is optional from FreeRTOS V7.3.0, and we use in SJOne, V8.1.2 */);
            // clear the INT
            LPC_GPIOINT->IO2IntClr |= (1 << i);
        }
    }
}
}
#endif

#define PORT_TO_IDX(p) (p == 0 ? eint3_monitor_PORT0_IDX : eint3_monitor_PORT2_IDX)

bool eint3_monitoring_hw_int::RegisterINTCallback(tPINInfo* apPinInfo)
{
    if(!apPinInfo || !apPinInfo->cb || !(apPinInfo->nPort == 0 || apPinInfo->nPort == 2)
            || !(apPinInfo->nPIN >= 0 && apPinInfo->nPIN < 32)
            || !(apPinInfo->nAssociatedINTTypes == kINT_EdgeTrig_R)/** currently we don't support any other INT */)
        return false;

    regTable[PORT_TO_IDX(apPinInfo->nPort)][apPinInfo->nPIN] = *apPinInfo;

    LOGD("registered the user callback function for P%d.%d\n", apPinInfo->nPort, apPinInfo->nPIN);
    return true;
}

bool eint3_monitoring_hw_int::run(void* p)
{
    tPINInfo pinInfo;
    //wait for entry in xQueueEINT3 indefinitely - until an EINT3 receives an INT
    LOGD("waiting for EINT3\n");
    while(xQueueReceive(xQueueEINT3, &pinInfo, portMAX_DELAY) == pdTRUE)
    {
        LOGD("got EINT3 at P%d.%d, call registered user callback if any\n",
                pinInfo.nPort, pinInfo.nPIN);
        /* got the pinInfo, so raise the user function Callback here
         * User can block on this task without compromising RTOS principle,
         * still, if this func is going to block for more than 4 queued INTs,
         * new INTs will be cleared without any notice
         */
        regTable[PORT_TO_IDX(pinInfo.nPort)][pinInfo.nPIN].ullTS = pinInfo.ullTS;
        if(regTable[PORT_TO_IDX(pinInfo.nPort)][pinInfo.nPIN].cb
           && (regTable[PORT_TO_IDX(pinInfo.nPort)][pinInfo.nPIN].nAssociatedINTTypes & pinInfo.nAssociatedINTTypes)) //INT is registered for by user
        {
            (regTable[PORT_TO_IDX(pinInfo.nPort)][pinInfo.nPIN].cb)(&regTable[PORT_TO_IDX(pinInfo.nPort)][pinInfo.nPIN]);
        }
    }
    return true; //Queue receive failed - which shall never happen, yet task shall never die, so return true
}

eint3_monitoring_hw_int::eint3_monitoring_hw_int() : scheduler_task("hweint3_t", 512 * 8, PRIORITY_HIGH, NULL)
{
    /** clear the matrix; c++, so row-major order as contiguous mem */
    memset(regTable, 0, sizeof(tPINInfo) * 2 * 32);
    xQueueEINT3 = xQueueCreate(eint3_monitor_MAX_NUM_OF_INTERRUPTS_TO_Q, sizeof(tPINInfo));
    LOGD("constructing the EINT3 INT monitor\n");
    /** enable rising edge interrupt
     * on all the P0 and P2 pins
     * we are enabling INT for all pins on which INT gen is supported
     * As we know, only P0 pins 0 -30 and P2 pins 0 - 13 are
     * supported pins in LPC17xx GPIO category to generate interrupts
     * Still, ours is a 80-pin package and thus certain pins among the above
     * also does not support INT generation
     * So, for this homework assignment, am implementing EINT3 monitoring
     * only for pins:
     * P0 0 - 2 //unavailable pins among (0-30): 4,5,19,20,21,23,24,27,28
     * P2 0 - 2 //unavailable pins among (0-13): 11,12,13
     * TODO support monitoring all relevant pins
     * TODO_DONE understand why hweint3_task is crashing when say, am supporting pin P0.2!
     * - Was crashing because P0.2 was wired to TXD0 -
     * UART0- which was used in a lot of places and when signals were out on that PIN
     * it triggered a ISR and eventually came into this task with the Queue,
     * but a bug: where I was not memset'ing regTable caused a random junk address in P0.2's cb pointer
     * causing the crash where cb was used to call non-existent function
     * Fix: memset regTable
     *  */
    NVIC_EnableIRQ(EINT3_IRQn);
    LPC_GPIOINT->IO0IntEnR |= 0x7;//0x7fffffff; //(30:0)
    LPC_GPIOINT->IO2IntEnR |= 0x7;//0x3fff; //(13:0)
    /** TODO we don't support other INT types yet */
    // register the EINT3 INT handler
    isr_register(EINT3_IRQn, EINT3_IRQHandlerTest);
}
eint3_monitoring_hw_int::~eint3_monitoring_hw_int()
{
    vQueueDelete(xQueueEINT3);
}
