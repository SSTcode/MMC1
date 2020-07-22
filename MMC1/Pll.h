#pragma once
#pragma once

#ifndef PLL_H_
#define PLL_H_
#include "Controllers.h"
//#include "CLA_files/cla_shared.h"
//#include "CLA_files/CLA_math/CLAmath.h"

enum PLL_state_enum
{
	omega_preinit,
	omega_init,
	PLL_check,
	PLL_active
};

struct PLL_struct {
	struct SOGI_struct SOGI_alf;
	struct SOGI_struct SOGI_bet;
	struct PI_struct PI;
	float Ts;
	float theta_1;
	float theta_2;
	float theta_3;
	float w;
	float w_filter1;
	float w_filter2;
	float f;
	float f_filter1;
	float f_filter2;
	float RDY;
	float ERR;
	float omega_nominal;
	enum PLL_state_enum state, state_last;
};

extern struct PLL_struct PLL;

void PLL_calc(float enable);

#endif /* PLL_H_ */