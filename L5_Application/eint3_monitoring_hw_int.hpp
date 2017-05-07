/*
 * eint3_monitoring_hw_int.h
 *
 *  Created on: Mar 13, 2017
 *      Author: unnik
 */

#ifndef L5_APPLICATION_EINT3_MONITORING_HW_INT_HPP_
#define L5_APPLICATION_EINT3_MONITORING_HW_INT_HPP_

#include "scheduler_task.hpp"
#include "soft_timer.hpp"

/** if pfn_INT_callback user-function is going to block for more than these many queued INTs,
 * new INTs will be cleared without any notice */
#define eint3_monitor_MAX_NUM_OF_INTERRUPTS_TO_Q 4

#define eint3_monitor_PORT0 0
#define eint3_monitor_PORT2 2

/** The API here, will need to implement a solution where
 * whenever there is an INTERRUPT (say Rising edge INT) on
 * a registered: (Port_PIN) call the corresponding registered
 * callback
 * */

/**
 * @brief This enum helps generate a bitmap of INT Types
 * which could be associated with a PIN with #tPINInfo
 *  */
typedef enum
{
    kINT_EdgeTrig_R = 1 << 0,
    kINT_EdgeTrig_F = 1 << 1
    /** TODO add support for other INT Types */
}eINTType;

/** The callback the user can register with
 * RegisterINTCallback() */
typedef struct PINInfo tPINInfo;
typedef void (*pfn_INT_callback)(tPINInfo* apPinInfo);

struct PINInfo
{
    uint8_t nPort;
    uint8_t nPIN;
    eINTType nAssociatedINTTypes; /**< bitmap */
    pfn_INT_callback cb;
    void* pUserData;
    uint64_t ullTS; /**< Switch press timestamp */
};

/** TODO learn and use singleton class here */
class eint3_monitoring_hw_int : public scheduler_task
{
public:
    eint3_monitoring_hw_int();
    ~eint3_monitoring_hw_int();
    /**
     * @brief Registers Callback for a PIN and an INTERRUPT Type
     * Register for all INT types at once using the bitmap: nAssociatedINTTypes
     * Registering callback with subsequent call for another type will overwrite the existing callback registration.
     */
    bool RegisterINTCallback(tPINInfo* apPinInfo);
    bool run(void* p);
private:
    tPINInfo regTable[2][32];
};

#endif /* L5_APPLICATION_EINT3_MONITORING_HW_INT_HPP_ */
