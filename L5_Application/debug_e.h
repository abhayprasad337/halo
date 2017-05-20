/*
 * debug_e.h
 *
 *  Created on: Apr 8, 2017
 *      Author: unnik
 */

#ifndef L5_APPLICATION_DEBUG_E_H_
#define L5_APPLICATION_DEBUG_E_H_
#include "printf_lib.h"
// #define DEBUG_E in your c/cpp file before #include<debug_e.h> to enable debug logs
#ifdef DEBUG_E
#define LOGD(fmt , ...) \
    u0_dbg_printf("[DEBUG %s %s %d] " fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)

#else
#define LOGD(...)
#endif

// #define VERBOSE_E in your c/cpp file before #include<debug_e.h> to enable debug logs
#ifdef VERBOSE_E
#define LOGV(fmt , ...) \
    u0_dbg_printf("[INFO %s %s %d] " fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define LOGV(...)
#endif

//error shall be ON always
#define LOGE(fmt , ...) \
    u0_dbg_printf("[ERROR %s %s %d] " fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)

#endif /* L5_APPLICATION_DEBUG_E_H_ */
