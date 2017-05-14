/*
 * ultrasonic_sensor_interface.hpp
 *
 *  Created on: Apr 23, 2017
 *      Author: Revthy
 */

#ifndef L5_APPLICATION_SOURCE_ULTRASONIC_SENSOR_INTERFACE_HPP_
#define L5_APPLICATION_SOURCE_ULTRASONIC_SENSOR_INTERFACE_HPP_

#include "scheduler_task.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include "soft_timer.hpp"
#include "exp_halo_api.h"

typedef enum
{
	safe=0,
	warning=1,
	alert=2
}warninglevel;

class USS_PeriodicTriggerTask : public scheduler_task
{
	public:
	USS_PeriodicTriggerTask(uint8_t priority);
	USS_PeriodicTriggerTask(uint8_t priority,size_t pMsgHandler);
	bool init(void);
    bool run(void*);
    void runStateMachine(void);
    size_t   MsgHandler;




	private:
    uint32_t mstartTime;
    uint32_t mendTime;
    eHalo_Mod_USS_WarningLevels mstatePrev ;
    eHalo_Mod_USS_WarningLevels mstateCurrent ;
    volatile uint32_t mEchoDuration;
    volatile float mdistance;

    SoftTimer mxTimerPeriodicTrigger;
    QueueHandle_t mxUSSCmdQ;
};

#endif /* L5_APPLICATION_SOURCE_ULTRASONIC_SENSOR_INTERFACE_HPP_ */
