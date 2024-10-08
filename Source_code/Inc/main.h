/*
 * main.h
 *
 *  Created on: Aug 21, 2024
 *      Author: HQLap
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "bno055.h"
#include "bno055_conf.h"
#include "orientation.h"

//sys_clk source
#define SYS_CLOCK_FREQ_50_MHZ   50
#define SYS_CLOCK_FREQ_84_MHZ   84
#define SYS_CLOCK_FREQ_120_MHZ  120
#define SYS_CLOCK_FREQ_168_MHZ	168

//Angular Rate energy
#define WINDOW_SIZE 17
#define SIGMA_OMEGA 1.0f
#define GAMMA_OMEGA 0.5f

#define MAG_REF_NORM    45.773f
#define DIP_ANGLE_REF   31.9635f
#define DIP_ANGLE_TH    15.0f
#define VRX             0.0f
#define VRY             -1.0f
#define VRZ             0.0f

#define PI								3.142857f
#define RAD_TO_DEG						57.2957795131f 		// 180/pi

#define TRUE  1
#define FALSE 0

#define BTN_PRESSED TRUE

#endif /* MAIN_H_ */
