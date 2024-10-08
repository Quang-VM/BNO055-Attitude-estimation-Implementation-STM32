/*
 *
 *
 *  Created on: Sep 17, 2024
 *      Author: HQLap
 */

#include "main.h"

float sign(float x) {
    return (x > 0) - (x < 0);
}

float clip(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * Multiply two quaternions.
 *
 * This function computes the product of two quaternions given their components,
 * defined by p1, p2, p3, p4 for the first quaternion and q1, q2, q3, q4 for the second quaternion.
 *
 * The quaternion product is computed using the formula:
 * pq = p1*q1 - p2*q2 - p3*q3 - p4*q4,
 *       p1*q2 + p2*q1 + p3*q4 - p4*q3,
 *       p1*q3 - p2*q4 + p3*q1 + p4*q2,
 *       p1*q4 + p2*q3 - p3*q2 + p4*q1]
 *
 * The result quaternion's components are returned via pointer parameters.
 *
 * @param p1 Real part of the first quaternion.
 * @param p2 First imaginary part of the first quaternion.
 * @param p3 Second imaginary part of the first quaternion.
 * @param p4 Third imaginary part of the first quaternion.
 * @param q1 Real part of the second quaternion.
 * @param q2 First imaginary part of the second quaternion.
 * @param q3 Second imaginary part of the second quaternion.
 * @param q4 Third imaginary part of the second quaternion.
 * @param pq1 Pointer to store the real part of the resulting quaternion.
 * @param pq2 Pointer to store the first imaginary part of the resulting quaternion.
 * @param pq3 Pointer to store the second imaginary part of the resulting quaternion.
 * @param pq4 Pointer to store the third imaginary part of the resulting quaternion.
 */
void q_prod(float p1, float p2, float p3, float p4,
            float q1, float q2, float q3, float q4,
            float *pq1, float *pq2, float *pq3, float *pq4) {
    // Calculate the real part of the resulting quaternion
    *pq1 = p1 * q1 - p2 * q2 - p3 * q3 - p4 * q4;

    // Calculate the first imaginary part of the resulting quaternion
    *pq2 = p1 * q2 + p2 * q1 + p3 * q4 - p4 * q3;

    // Calculate the second imaginary part of the resulting quaternion
    *pq3 = p1 * q3 - p2 * q4 + p3 * q1 + p4 * q2;

    // Calculate the third imaginary part of the resulting quaternion
    *pq4 = p1 * q4 + p2 * q3 - p3 * q2 + p4 * q1;
}


/*
 * Quaternion from a Direction Cosine Matrix (DCM) using Chiaverini's algebraic method.
 * The function takes a 3x3 DCM matrix and converts it into a quaternion.
 *
 * Inputs:
 * - R11, R12, R13, R21, R22, R23, R31, R32, R33: Elements of the 3x3 DCM matrix.
 * - q1, q2, q3, q4: Pointers to store the computed quaternion components (q1 = W, q2 = X, q3 = Y, q4 = Z).
 *
 * Outputs:
 * - Quaternion (q1, q2, q3, q4) representing the rotation corresponding to the input DCM matrix.
 */
void dcm2quat(float R11, float R12, float R13,
              float R21, float R22, float R23,
              float R31, float R32, float R33,
              float *q1, float *q2, float *q3, float *q4) {

	float norm;
    *q1 = 0.5f * sqrtf(clip(R11 + R22 + R33, -1.0f, 3.0f) + 1.0f);
    *q2 = 0.5f * sign(R32 - R23) * sqrtf(clip(R11 - R22 - R33, -1.0f, 1.0f) + 1.0f);
    *q3 = 0.5f * sign(R13 - R31) * sqrtf(clip(R22 - R33 - R11, -1.0f, 1.0f) + 1.0f);
    *q4 = 0.5f * sign(R21 - R12) * sqrtf(clip(R33 - R11 - R22, -1.0f, 1.0f) + 1.0f);

    norm = sqrtf((*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3) + (*q4) * (*q4));
    *q1 /= norm;
    *q2 /= norm;
    *q3 /= norm;
    *q4 /= norm;
}

/**
 * @brief Converts a quaternion to ZYX Euler angles (yaw, pitch, roll) in degrees.
 * @param sensorData Pointer to the BNO055_Sensors_t structure containing the quaternion.
 * where phi is a rotation around X, theta around Y and psi around Z.
 */
void quat2EUangle(float q0, float q1, float q2, float q3, float *yaw, float *pitch, float *roll ){

	// axulirary variables to avoid reapeated calcualtions
	float twoq0 = 2.0f * q0;
	float twoq1 = 2.0f * q1;
	float twoq2 = 2.0f * q2;
	//float twoq3 = 2.0f * q3;
	float q0q0 = q0 * q0;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;

	float a12, a22, a31, a32, a33;

	// Convert quaternions to Euler angles
	a12 = twoq1*q2 + twoq0*q3;
	a22 = q0q0 + q1q1 - q2q2 - q3q3;
	a31 = twoq0*q1 + twoq2*q3;
	a32 = twoq1*q3 - twoq0*q2;
	a33 = q0q0 - q1q1 - q2q2 + q3q3;


	*pitch = -asinf(a32);
	*roll  = atan2f(a31, a33);
	*yaw   = atan2f(a12, a22);

	*pitch *= RAD_TO_DEG;
	*yaw   *= RAD_TO_DEG;
	*roll  *= RAD_TO_DEG;

	*yaw   -= 1.9755f; // Declination
	if(*yaw < 0) {
		*yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	}
}





