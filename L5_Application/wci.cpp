/*
 * wci.c
 *
 *  Created on: 15-May-2017
 *      Author: Vishal
 */

#include <stdlib.h>
#include <wci.hpp>
#include "tasks.hpp"
#include "L4_IO/wireless/wireless.h"
#include "L4_IO/wireless/src/mesh.h"
#include "exp_halo_api.h"

/**
 * API to send wireless message to Board-2
 *
 * Arguments:
 * 1) WCI context handle
 * 2) tHalo_Msg type message
 *
 */

size_t gHalo_WCI_SendMsg(size_t axhWCI, tHalo_Msg* axpMsg)
{
	mesh_packet_t pkt;
	tHalo_WCI *xhWCI = (tHalo_WCI *)axhWCI;

	wireless_form_pkt(&pkt, xhWCI->uHalo_Dest_Addr, mesh_pkt_ack, xhWCI->uHalo_hops, 1, axpMsg, sizeof(tHalo_Msg));

	wireless_send_formed_pkt(&pkt);

	while(!(wireless_get_ack_pkt(&pkt, 200) && (xhWCI->uHalo_Dest_Addr == pkt.nwk.src)))
	{
		wireless_send_formed_pkt(&pkt);
	}

	return axhWCI;
}



/**
 * API to initialize WCI module
 * It supports TX as well as RX mode
 */


size_t gHalo_WCI_Init(tHalo_Ctx* axpHCtx)
{

	tHalo_WCI* xhWCI;
	xhWCI = (tHalo_WCI *)malloc(sizeof(tHalo_WCI));

	xhWCI->uHalo_Brdcst_Data = 0x55;
	xhWCI->uHalo_hops			   = 1;

	xhWCI->uHalo_Node_Addr = (axpHCtx->xnBoardID == kHalo_BoardID_Tx) ? WCI_B1_ADDR : WCI_B2_ADDR;
	xhWCI->uHalo_Dest_Addr = (axpHCtx->xnBoardID == kHalo_BoardID_Tx) ? WCI_B2_ADDR : WCI_B1_ADDR;

	xhWCI->xpHCtx = axpHCtx->xhMHI;

	mesh_set_node_address(xhWCI->uHalo_Node_Addr);

	if(axpHCtx->xnBoardID == kHalo_BoardID_Rx)
	{
		scheduler_add_task(new WCI_rcv(PRIORITY_HIGH, xhWCI));
	}

	return (size_t)xhWCI;

}



