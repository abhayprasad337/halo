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


class US_PeriodicTrigger : public scheduler_task
{
	public:
	US_PeriodicTrigger(uint8_t priority);
	bool init(void);
    bool run(void*);


	private:
    SoftTimer mxTimerPeriodicTrigger;
    SemaphoreHandle_t mxSemaphoreTriggerSignal;
    bool mstartTrigger;

};

class US_EchoDetect : public scheduler_task
{
	public:
	US_EchoDetect(uint8_t priority);
	bool init(void);
    bool run(void*);


	private:
    uint64_t mEchoDuration;
    uint64_t mstartTime;
    uint64_t mendTime;
    float mdistance;
    SemaphoreHandle_t mxSemaphoreWarningL1;
    SemaphoreHandle_t mxSemaphoreTriggeredSignal;
    SemaphoreHandle_t mxSemaphoreEchoRcvdSignal;
    EventGroupHandle_t maliveEchoDetect;

};


#endif /* L5_APPLICATION_SOURCE_ULTRASONIC_SENSOR_INTERFACE_HPP_ */
