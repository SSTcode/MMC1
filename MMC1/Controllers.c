#include "stdafx.h"
#include "Controllers.h"

#define myMATH_PI 3.1415926535897932384626433832795
#define myMATH_SQRT3 1.7320508075688772935274463415059
#define myMATH_SQRT2 1.4142135623730950488016887242097

const float MATH_PI = myMATH_PI;
const float MATH_2PI = myMATH_PI * 2.0;
const float MATH_2PI_3 = myMATH_PI * 2.0 / 3.0;
const float MATH_PI_3 = myMATH_PI / 3.0;
const float MATH_1_2PI = 1.0 / (myMATH_PI * 2.0);
const float MATH_1_PI = 1.0 / myMATH_PI;
const float MATH_1_3 = 1.0 / 3.0;
const float MATH_1_SQRT3 = 1.0 / myMATH_SQRT3;
const float MATH_1_SQRT2 = 1.0 / myMATH_SQRT2;
const float MATH_SQRT2_3 = myMATH_SQRT2 / 3.0;
const float MATH_SQRT3_2 = myMATH_SQRT3 / 2.0;
const float MATH_SQRT2 = myMATH_SQRT2;
const float MATH_SQRT3 = myMATH_SQRT3;
const float MATH_2_3 = 2.0 / 3.0;

void SOGI_calc(struct SOGI_struct* SOGI, float input, float w)
{
	float qx_mod = SOGI->qx + 0.5f * SOGI->x * w * SOGI->Ts;
	float x_err = SOGI->input_err - qx_mod;
	SOGI->x += x_err * w * SOGI->Ts;
	SOGI->qx = qx_mod + 0.5f * SOGI->x * w * SOGI->Ts;
	SOGI->input_err = input - SOGI->x;
}

void PI_antiwindup_fast(struct PI_struct* PI, float error)
{
	PI->proportional = PI->Kp * error;
	PI->integrator += PI->proportional * PI->Ts / PI->Ti;
	PI->out = PI->integrator + PI->proportional;
	if (PI->out > PI->lim_H)
	{
		PI->out = PI->lim_H;
		PI->integrator = PI->lim_H - PI->proportional;
	}
	if (PI->out < PI->lim_L)
	{
		PI->out = PI->lim_L;
		PI->integrator = PI->lim_L + PI->proportional;
	}
}

void PI_antiwindup(struct PI_struct* PI, float error)
{
	PI->integrator_last = PI->integrator;
	PI->proportional = PI->Kp * error;
	PI->integrator += PI->Ts * error * PI->Kp / PI->Ti;
	PI->out = PI->integrator + PI->proportional;
	if (PI->out > PI->lim_H)
	{
		PI->out = PI->lim_H;
		if ((PI->integrator < PI->lim_H) || (PI->integrator > PI->integrator_last))
		{
			PI->integrator = PI->integrator_last;
		}
	}
	if (PI->out < PI->lim_L)
	{
		PI->out = PI->lim_L;
		if ((PI->integrator > PI->lim_L) || (PI->integrator < PI->integrator_last))
		{
			PI->integrator = PI->integrator_last;
		}
	}
}

void PR_calc(struct PR_struct* PR, float error)
{
	PR->y0 += PR->x1 * PR->Ts;
	PR->x0 += PR->w * PR->w * PR->y0 * PR->Ts;
	PR->out = PR->y0 * PR->Ki + error * PR->Kp;
	PR->x1 = error - PR->x0;


}

void PR_calc_imp(struct PR_struct* PR, float error)
{
	float b1 = cosf(PR->w * PR->Ts);
	PR->y0 = PR->Ts * error - b1 * (PR->x0 - 2.0f * PR->x2) - PR->x1;
	PR->x0 = PR->Ts * error;
	PR->x1 = PR->x2;
	PR->x2 = PR->y0;
	PR->out = PR->y0 * PR->Ki + error * PR->Kp;
}
