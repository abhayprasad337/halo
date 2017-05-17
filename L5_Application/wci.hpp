/*
 * wci.h
 *
 *  Created on: 15-May-2017
 *      Author: Vishal
 */

#ifndef L5_APPLICATION_WCI_HPP_
#define L5_APPLICATION_WCI_HPP_

#include <stddef.h>
#include <stdint.h>
#include "tasks.hpp"
#include "exp_halo_api.h"

#define WCI_B1_ADDR		101
#define WCI_B2_ADDR		100


typedef struct
{
	uint8_t uHalo_Dest_Addr;
	uint8_t uHalo_Node_Addr;
	uint8_t uHalo_Brdcst_Data;
	uint8_t uHalo_hops;
	size_t  xpHCtx;

}tHalo_WCI;


/** @{ WCI API */
/**
 * @brief Initialize WCI module
 */
size_t gHalo_WCI_Init(tHalo_Ctx* axpHCtx);


/**
 * @brief This API shall synchronously send out the supplied
 * tMsg* object
 * @param axpMsg[IN] the message to be sent to the remote
 */
size_t gHalo_WCI_SendMsg(size_t axhWCI, tHalo_Msg* axpMsg);
/** @} WCI API */


class WCI_rcv : public scheduler_task
{

	tHalo_WCI* mxhWCI;

    public:

        WCI_rcv (uint8_t priority, tHalo_WCI* xhWCI) : scheduler_task("WCI_rcv", 2048, priority)
        {
        	mxhWCI = xhWCI;
        }

        bool run(void *p)
        {

        	mesh_packet_t pkt;
			tHalo_Msg xpMsg;


            if (wireless_get_rx_pkt(&pkt, portMAX_DELAY))
            {

            		wireless_deform_pkt(&pkt, 1, &xpMsg, sizeof(tHalo_Msg));
					gHalo_MHI_BroadCast(mxhWCI->xpHCtx, &xpMsg);

            }


            vTaskDelay(1000);
            return true;
        }
};


#endif /* L5_APPLICATION_WCI_HPP_ */
