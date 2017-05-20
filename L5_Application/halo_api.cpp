/*
 * halo_api.cpp
 *
 *  Created on: May 14, 2017
 *      Author: unnik
 */

#define DEBUG_E
#define VERBOSE_E
#include "debug_e.h"
#include "exp_halo_api.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stdlib.h"

/** Maximum modules/queues into MHI possible */
#define MHI_INPUT_Q_COUNT (3)
/**
 * Delay for enqueue shall be zero - as this condition itself is undesirable
 * and should have not occured in the first place - where the Q is full.
 * */

#define Q_ENQ_DELAY (0)
/** priority implementation is based on the index into our xpQs[] array
 * lower indexes are higher priority queues
 * if you have 2 queues of same priority the indexes shall be adjacent
 * to each other - say 0 and 1 (so logically 0 and 1 will be processed
 * in sequence - even if queue 0 is technically having higher priority in
 * the sequence)
 *  */
#define MHI_Q_USS (0)
#define MHI_Q_MAE (1)
#define MHI_Q_UIO (2)

/** Individual Queue Lengths */
#define Q_FROM_MAE_LENGTH (2)
#define Q_FROM_USS_LENGTH (2)
#define Q_FROM_UIO_LENGTH (2)
#define Q_FROM_ALL_COMBINED (Q_FROM_MAE_LENGTH + Q_FROM_USS_LENGTH + Q_FROM_UIO_LENGTH)


typedef struct
{
    QueueHandle_t xpQs[MHI_INPUT_Q_COUNT];
    /** We have a Queue set with the above 3 queues added in it
     * And we in MHI shall block on all of these queues simultaneously
     * and process the messages coming into these queues
     * one at a time according to the priority associated with the queue
     * using xQueueSelectFromSet()
     *
     * Basically MHI is implementing a logical-priority Queue
     * with xhQSet
     * */
    QueueSetHandle_t xhQSet;
    tHalo_Ctx* xpHCtx;
}tMHI_Ctx;

void task_mhi(void* p)
{
    tMHI_Ctx* pMHICtx = (tMHI_Ctx*)p;
    QueueSetMemberHandle_t xhActiveQ;
    uint8_t i;
    tHalo_Msg msg;
    while(1)
    {
        xhActiveQ = xQueueSelectFromSet(pMHICtx->xhQSet, portMAX_DELAY);
        if(!xhActiveQ)
        {
            LOGE("xQueueSelectFromSet failed\n");
            continue;
        }
        LOGD("QSelectFrpmSet out\n");
        /** here we know one or more queues have data
         * Just peek on all the queues and receive from
         * the highest priority queue
         *  */
        for(i = 0; i < MHI_INPUT_Q_COUNT; i++)
        {
            if(xQueueReceive(pMHICtx->xpQs[i], &msg, 0) == pdTRUE)
            {
                LOGD("WCI tx\n");
                gHalo_WCI_SendMsg(pMHICtx->xpHCtx->xhWCI, &msg);
                LOGD("WCI tx done\n");
                /** now break the for loop and
                 * do this loop all over again as we
                 * need higher priority Q dequeue first! */
                break;
            }
        }
    }
}

/** @{ API to use the Message Handler Interface */
size_t gHalo_MHI_Init(tHalo_Ctx* axpHCtx)
{
    tMHI_Ctx* pMHICtx = (tMHI_Ctx*)calloc(1, sizeof(tMHI_Ctx));
    BaseType_t ret;

    if(!pMHICtx)
        goto cleanup;

    pMHICtx->xpQs[MHI_Q_MAE] = xQueueCreate(Q_FROM_MAE_LENGTH, sizeof(tHalo_Msg));
    pMHICtx->xpQs[MHI_Q_USS] = xQueueCreate(Q_FROM_USS_LENGTH, sizeof(tHalo_Msg));
    pMHICtx->xpQs[MHI_Q_UIO] = xQueueCreate(Q_FROM_UIO_LENGTH, sizeof(tHalo_Msg));
    pMHICtx->xhQSet = xQueueCreateSet(Q_FROM_ALL_COMBINED);

    xQueueAddToSet(pMHICtx->xpQs[MHI_Q_MAE], pMHICtx->xhQSet);
    xQueueAddToSet(pMHICtx->xpQs[MHI_Q_USS], pMHICtx->xhQSet);
    xQueueAddToSet(pMHICtx->xpQs[MHI_Q_UIO], pMHICtx->xhQSet);

    pMHICtx->xpHCtx = axpHCtx;
    /** MHI will be of same priority as other tasks to maintain fareness among the
     * equal priority tasks (USS and MAE need equal computing time; priority is
     * handled with the priority queue block inside MHI)
     *  */
    ret = xTaskCreate(task_mhi, "mhi", 512 * 10, pMHICtx, PRIORITY_MEDIUM, NULL);
    if(ret != pdTRUE)
    {
        free(pMHICtx);
        return (size_t)NULL;
    }

    cleanup:

    return (size_t)pMHICtx;
}

bool gHalo_MHI_BroadCast(size_t axhMHI, tHalo_Msg* axpMsg)
{
    tMHI_Ctx* pMHICtx = (tMHI_Ctx*)axhMHI;
    LOGD("DEBUGME\n");
    bool ret = true;
    if(!pMHICtx)
    {
        LOGE("BCAST handle NULL\n");
        ret = false;
        goto cleanup;
    }
    if(pMHICtx->xpHCtx->xnBoardID == kHalo_BoardID_Tx)
    {
        switch(axpMsg->xnSrc)
        {
            case kHalo_MsgSrc_Mod_MAE:
                ret = (xQueueSend(pMHICtx->xpQs[MHI_Q_MAE], axpMsg, Q_ENQ_DELAY) == pdTRUE);
                break; /**< kHalo_MsgSrc_Mod_MAE */
            case kHalo_MsgSrc_Mod_USS:
                ret = (xQueueSend(pMHICtx->xpQs[MHI_Q_USS], axpMsg, Q_ENQ_DELAY) == pdTRUE);
                break; /**< kHalo_MsgSrc_Mod_USS */
            case kHalo_MsgSrc_Mod_UIO:
                ret = (xQueueSend(pMHICtx->xpQs[MHI_Q_UIO], axpMsg, Q_ENQ_DELAY) == pdTRUE);
                break; /**< kHalo_MsgSrc_Mod_UIO */
            default:
                ret = false;
                break;
        }
    }
    else if(pMHICtx->xpHCtx->xnBoardID == kHalo_BoardID_Rx)
    {
        LOGD("ADA SHW\n");
        gHalo_ADA_Show(pMHICtx->xpHCtx->xhADA, axpMsg);
    }

    LOGD("DEBUGME %d\n", pMHICtx->xpHCtx->xnBoardID);

    cleanup:

    return ret;
}
/** @} API to use the Message Handler Interface */

tHalo_Ctx* gHalo_Init(eHalo_BoardID axnBoardID)
{
    tHalo_Ctx* pHCtx = (tHalo_Ctx*)calloc(1, sizeof(tHalo_Ctx));
    pHCtx->xnBoardID = axnBoardID;
    pHCtx->xhMHI = gHalo_MHI_Init(pHCtx);
    pHCtx->xhWCI = gHalo_WCI_Init(pHCtx);
    LOGV("gHalo_Init; %p %p\n", pHCtx->xhWCI, pHCtx->xhMHI);
    if(pHCtx->xnBoardID == kHalo_BoardID_Tx)
    {
        LOGD("DEBUGME\n");
        pHCtx->xhMAE = gHalo_MAE_Init(pHCtx);
        pHCtx->xhUSS = gHalo_USS_Init(pHCtx);
        pHCtx->xhUIO = gHalo_UIO_Init(pHCtx);
    }
    else if(pHCtx->xnBoardID == kHalo_BoardID_Rx)
    {
        pHCtx->xhADA = gHalo_ADA_Init(pHCtx);
    }
    return pHCtx;
}
