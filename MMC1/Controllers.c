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

void PI_tustin(struct PI_struct* PI_tus,float error)
{
	float out_temp = 0.0;
	out_temp = (PI_tus->Ki * PI_tus->Ts)*0.5f * (error + PI_tus->error_prev) + PI_tus->out_prev;
	if (out_temp - error * PI_tus->Kp > PI_tus->lim_H) out_temp = PI_tus->lim_H - error * PI_tus->Kp;
	if (out_temp - error * PI_tus->Kp < PI_tus->lim_L) out_temp = PI_tus->lim_L - error * PI_tus->Kp;
	PI_tus->out_prev = out_temp;
	PI_tus->error_prev = error;
	PI_tus->out = out_temp + error*PI_tus->Kp;
}

void PI_tustin_3ph(struct PI_struct* PI_tus, float error[3])
{
	int i;
	float out_temp[3] = { 0.0,0.0,0.0 };
	for (i = 0; i < 3; i++) {
		out_temp[i] = (PI_tus->Ki * PI_tus->Ts) * 0.5f * (error[i] + PI_tus->error_prev_3ph[i]) + PI_tus->out_prev_3ph[i];
		if (out_temp[i] - error[i] * PI_tus->Kp > PI_tus->lim_H) out_temp[i] = PI_tus->lim_H - error[i] * PI_tus->Kp;
		if (out_temp[i] - error[i] * PI_tus->Kp < PI_tus->lim_L) out_temp[i] = PI_tus->lim_L - error[i] * PI_tus->Kp;
		PI_tus->out_prev_3ph[i] = out_temp[i];
		PI_tus->error_prev_3ph[i] = error[i];
		PI_tus->out_3ph[i] = out_temp[i] + error[i] * PI_tus->Kp;
	}
}

void xy2Dec(struct DEC_struct* DEC, float xy[6])
{																								
	DEC->o[0] = 0.166666667f * (2.0f * xy[0] + 2.0f * xy[3] - xy[1] - xy[4] - xy[2] - xy[5]);
	DEC->o[1] = 0.166666667f * (-1.0f* xy[0] - xy[3] + 2.0f * xy[1] + 2.0f * xy[4] - xy[2] - xy[5]);
	DEC->o[2] = 0.166666667f * (-1.0f* xy[0] - xy[3] - xy[1] - xy[4] + 2.0f * xy[2] + 2.0f * xy[5]);

	DEC->s    = 0.166666667f * (       xy[0] - xy[3] + xy[1] - xy[4] + xy[2] - xy[5]);

	DEC->z[0] = 0.166666667f * (2.0f * xy[0] - 2.0f * xy[3] - xy[1] + xy[4] - xy[2] + xy[5]);
	DEC->z[1] = 0.166666667f * (-1.0f* xy[0] + xy[3] + 2.0f * xy[1] - 2.0f * xy[4] - xy[2] + xy[5]);
	DEC->z[2] = 0.166666667f * (-1.0f* xy[0] + xy[3] - xy[1] + xy[4] + 2.0f * xy[2] - 2.0f * xy[5]);

	DEC->m    = 0.166666667f * (       xy[0] + xy[1] + xy[2] + xy[3] + xy[4] + xy[5]);
}


void Dec2xy(struct DEC_struct* DEC, float o[3], float s, float z[3], float m)
{
	DEC->xy[0] = o[0] + s + z[0] + m; //pa
	DEC->xy[1] = o[1] + s + z[1] + m; //pb 
	DEC->xy[2] = o[2] + s + z[2] + m; //pc 
	DEC->xy[3] = o[0] - s - z[0] + m; //na 
	DEC->xy[4] = o[1] - s - z[1] + m; //nb
	DEC->xy[5] = o[2] - s - z[2] + m; //nc

}

