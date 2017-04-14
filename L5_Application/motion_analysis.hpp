/*
 * motion_analysis.hpp
 *
 *  Created on: Apr 9, 2017
 *      Author: unnik
 */

#ifndef L5_APPLICATION_MOTION_ANALYSIS_HPP_
#define L5_APPLICATION_MOTION_ANALYSIS_HPP_

/**
 * @fn void* xStartMotionAnalysis()
 * @brief This starts a new task continuously fetching the
 * accelerometer reading over the interested planes
 * Features:
 * 1) A small log dump
 */
void* xStartMotionAnalysis();

#endif /* L5_APPLICATION_MOTION_ANALYSIS_HPP_ */
