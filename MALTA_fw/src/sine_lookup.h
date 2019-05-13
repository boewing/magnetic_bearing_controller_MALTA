/*
 * sine_lookup.h
 *
 *  Created on: 10.05.2018
 *      Author: Dietschi
 */

/**
 * Example for a interpolated sine/cosine table lookup
 *
 * @file sin1.h
 * @author stfwi
 *
 */
#ifndef __SW__SIN1_H__
#define __SW__SIN1_H__
#ifdef  __cplusplus
extern "C" {
#endif

#include "math.h"
#include <sys/types.h>


#define Q15 (1.0/(double)((1<<15)-1))

/**
 * Sine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * sine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t sin1(int16_t angle);

float sin_lu(float angle);

/**
 * Cosine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * cosine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t cos1(int16_t angle);
float cos_lu(float angle);

#ifdef  __cplusplus
}
#endif
#endif

