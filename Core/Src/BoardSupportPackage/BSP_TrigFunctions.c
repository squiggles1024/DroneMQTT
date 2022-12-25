/*
 * BSP_TrigFunctions.c
 *
 *  Created on: Dec 2, 2022
 *      Author: evanl
 */

#include <stdint.h>
#include <math.h>
#include "cordic.h"
#include "BSP_TrigFunctions.h"

#define q31_to_f32(x) ldexp((int32_t) x, -31)

static const uint32_t cst_scale_tab[] =
{
    CORDIC_SCALE_0, CORDIC_SCALE_1,
	CORDIC_SCALE_2, CORDIC_SCALE_3,
	CORDIC_SCALE_4, CORDIC_SCALE_5,
	CORDIC_SCALE_6, CORDIC_SCALE_7
};

static const CORDIC_ConfigTypeDef CosCordicConfig = {
		.Function = CORDIC_FUNCTION_COSINE,
		.Precision = CORDIC_PRECISION_6CYCLES,
		.Scale = CORDIC_SCALE_0,
		.NbWrite = CORDIC_NBWRITE_1,
		.NbRead = CORDIC_NBREAD_1,
		.InSize = CORDIC_INSIZE_32BITS,
		.OutSize = CORDIC_OUTSIZE_32BITS,
};

static const CORDIC_ConfigTypeDef SinCordicConfig = {
		.Function = CORDIC_FUNCTION_SINE,
		.Precision = CORDIC_PRECISION_6CYCLES,
		.Scale = CORDIC_SCALE_0,
		.NbWrite = CORDIC_NBWRITE_1,
		.NbRead = CORDIC_NBREAD_1,
		.InSize = CORDIC_INSIZE_32BITS,
		.OutSize = CORDIC_OUTSIZE_32BITS,
};


static int f32_to_q31(double input)
{
	const float Q31_MAX_F = 0x0.FFFFFFp0F;
	const float Q31_MIN_F = -1.0F;
	float clamped = fmaxf(fminf(input, Q31_MAX_F), Q31_MIN_F);
	return (int)roundf(scalbnf(clamped,31));
}


float BSP_atan(float Ratio)
{
    float Scaled = Ratio;
    int32_t Input_Q31;
    int32_t n;
    uint32_t Scale;
    int32_t Output_Q31;

	CORDIC_ConfigTypeDef sConfig =
	{
			.Function = CORDIC_FUNCTION_ARCTANGENT,
			.Precision = CORDIC_PRECISION_6CYCLES,
			.NbWrite = CORDIC_NBWRITE_1,
			.NbRead = CORDIC_NBREAD_1,
			.InSize = CORDIC_INSIZE_32BITS,
			.OutSize = CORDIC_OUTSIZE_32BITS,
	};



	if(Ratio < -1.0f || Ratio > 1.0f)
	{
		Scaled = Ratio / pow(2, (int)(log2(fabs(Ratio)) + 1));
		n = log2(Ratio / Scaled);

		if(n >= 0 && n < 8)
		{
			Scale = cst_scale_tab[n];
		}

	}

	Input_Q31 = f32_to_q31(Scaled);
	sConfig.Scale = Scale;

	HAL_CORDIC_Configure(&hcordic, &sConfig);
	HAL_CORDIC_CalculateZO(&hcordic, &Input_Q31, &Output_Q31, 1, 0);
	return q31_to_f32(Output_Q31 << n) * M_PI;

}

float BSP_sin(float AngleInRads)
{
    int32_t InputQ31 = f32_to_q31(fmod(AngleInRads, 2.0f * M_PI) / (2.0f * M_PI)) << 1;
    int32_t Output_Q31;
    HAL_CORDIC_Configure(&hcordic, &SinCordicConfig);
    HAL_CORDIC_Calculate(&hcordic, &InputQ31, &Output_Q31, 1, 0);
    return q31_to_f32(Output_Q31);
}

float BSP_cos(float AngleInRads)
{
    int32_t InputQ31 = f32_to_q31(fmod(AngleInRads, 2.0f * M_PI) / (2.0f * M_PI)) << 1;
    int32_t Output_Q31;
    HAL_CORDIC_Configure(&hcordic, &CosCordicConfig);
    HAL_CORDIC_Calculate(&hcordic, &InputQ31, &Output_Q31, 1, 0);
    return q31_to_f32(Output_Q31);
}
