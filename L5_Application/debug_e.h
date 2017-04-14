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
#define LOGD(...) u0_dbg_printf(__VA_ARGS__)
#else
#define LOGD(...)
#endif

// #define VERBOSE_E in your c/cpp file before #include<debug_e.h> to enable debug logs
#ifdef VERBOSE_E
#define LOGV(...) u0_dbg_printf(__VA_ARGS__)
#else
#define LOGV(...)
#endif

//error shall be ON always
#define LOGE(...) u0_dbg_printf(__VA_ARGS__)

#endif /* L5_APPLICATION_DEBUG_E_H_ */
