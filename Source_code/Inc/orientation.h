/*
 * orientation.h
 *
 *  Created on: Sep 17, 2024
 *      Author: HQLap
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_
#include "main.h"


void q_prod(float p1, float p2, float p3, float p4,
            float q1, float q2, float q3, float q4,
            float *pq1, float *pq2, float *pq3, float *pq4);

void dcm2quat(float R11, float R12, float R13,
        	  float R21, float R22, float R23,
			  float R31, float R32, float R33,
			  float *q1, float *q2, float *q3, float *q4);

void quat2EUangle(float q0, float q1, float q2, float q3, float *yaw, float *pitch, float *roll);

//helper function
float sign(float x);
float clip(float value, float min, float max);



#endif /* ORIENTATION_H_ */
