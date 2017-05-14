#include <stddef.h>

/**
 * @dev-notes
 * Halo Project:
 * Is a smart helmet project with the following major modules:
 * 1) The Ultra Sound Sensor Module                     (USS)
 * [Revathy]
 * 2) The Motion Analyzer Engine Module                 (MAE)
 * [Abhay, Unni]
 * 3) The User IO Module (Switches and LEDs on a PCB)   (UIO)
 * [KP]
 * 4) The Alert Display Array Module                    (ADA)
 * [KP (LED array + buzzer(+Unni if required))]
 * 5) The Message Handler Interface                     (MHI)
 * [Unni]
 * 6) The Wireless Channel Interface                    (WCI)
 * [Vishal]
 */
/** 
 * @brief This enum specifies all the possible sources
 * for a @tHalo_Msg object in the system
 */
typedef enum
{
        kHalo_MsgSrc_Mod_USS, /**< from Ultra Sound */
        kHalo_MsgSrc_Mod_MAE, /**< from MAE */
        kHalo_MsgSrc_Mod_UIO, /**< from UIO */
        kHalo_MsgSrc_Mod_ADA  /**< from ADA */
}eHalo_MsgSrc;

/**
 * @brief This enum describes the various real-time Warning Levels
 * as identified by the USS (UltraSensor) module
 */
typedef enum
{
        kHalo_Mod_USS_WL_Warning, /**< Range: [2m, 5m] : 2m < current_displacement <= 5m */
        kHalo_Mod_USS_WL_Critical /**< Range: [0m, 2m] */
}eHalo_Mod_USS_WarningLevels;

/**
 * @brief This enum describes the Motion Analyzer Engine's 
 * observed events about the bike's motion
 */
typedef enum
{
        kHalo_Mod_MAE_EV_Moving,
        kHalo_Mod_MAE_EV_Stopping,
        kHalo_Mod_MAE_EV_Stopped
}eHalo_Mod_MAE_Event;

/**
 * @brief This enum describes the User IO Events from
 * the PCB board interface with bike-indicator switches
 */
typedef enum
{
        kHalo_Mod_UIO_EV_Left,
        kHalo_Mod_UIO_EV_Right,
}eHalo_Mod_UIO_Event;

typedef enum
{
        kHalo_BoardID_Tx, /**< The cycle mount */
        kHalo_BoardID_Rx  /**< The helmet mount */
}eHalo_BoardID;

/**
 * @brief USS base structure
 * [The object generated from USS shall be of this type]
 */
typedef struct
{
        eHalo_Mod_USS_WarningLevels xnWL;
}tHalo_Mod_UltraSoundSensor;

/**
 * @brief MAE base structure
 * [The object generated from MAE shall be of this type]
 */
typedef struct
{
        eHalo_Mod_MAE_Event xnEV;
}tHalo_Mod_MotionAnalyzerEngine;

/**
 * @brief UIO base structure
 * [The object generated from UIO shall be of this type]
 */
typedef struct
{
        eHalo_Mod_UIO_Event xnEV;
}tHalo_Mod_UserIO;

/**
 * @brief This object shall encapsulate 
 * a general structure type that any module shall generate
 * to use gHalo_MHI_* APIs 
 */
typedef struct
{
        eHalo_MsgSrc xnSrc;
        union
        {
                tHalo_Mod_UltraSoundSensor xUSS;
                tHalo_Mod_MotionAnalyzerEngine xMAE;
                tHalo_Mod_UserIO xUIO;
        };
}tHalo_Msg;

typedef struct
{
    size_t xhMHI;
    size_t xhUSS;
    size_t xhMAE;
    size_t xhUIO;
    size_t xhADA;
    eHalo_BoardID xnBoardID;
}tHalo_Ctx;

tHalo_Ctx* gHalo_Init();

/** @{ API to use the Message Handler Interface */
size_t gHalo_MHI_Init(tHalo_Ctx* axpHCtx);
bool gHalo_MHI_BroadCast(size_t axhMHI, tHalo_Msg* axpMsg);
/** @} API to use the Message Handler Interface */

/** @{ USS API */
size_t gHalo_USS_Init(tHalo_Ctx* axpHCtx);
/** @} USS API */

/** @{ MAE API */
size_t gHalo_MAE_Init(tHalo_Ctx* axpHCtx);
/** @} MAE API */

/** @{ UIO API */
size_t gHalo_UIO_Init(tHalo_Ctx* axpHCtx);
/** @} UIO API */

/** @{ ADA API */
/**
 * @brief Initialize ADA module
 */ 
size_t gHalo_ADA_Init(tHalo_Ctx* axpHCtx);
/**
 * @brief This API shall synchronously display the supplied
 * tMsg* object on the Alert Display Array module
 * @param axpMsg[IN] the message to be parsed for Alert Display
 */
size_t gHalo_ADA_Show(size_t axhADA, tHalo_Msg* axpMsg);
/** @} ADA API */

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
