/*
 * BSP_TrigFunctions.h
 *
 *  Created on: Dec 2, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_TRIGFUNCTIONS_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_TRIGFUNCTIONS_H_
#include <math.h>

#define RAD_2_DEG(x)     ((x) * 180 / M_PI)
#define DEG_2_RAD(x)     ((x) * M_PI / 180)

float BSP_atan(float Ratio);
float BSP_sin(float AngleInRads);
float BSP_cos(float AngleInRads);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_TRIGFUNCTIONS_H_ */
