#pragma once

#ifndef Compensator_H_
#define Compensator_H_
#include "Controllers.h"
#include "PLL.h"
//#include "CLA_files/cla_shared.h"
//#include "CLA_files/CLA_math/CLAmath.h"

enum Compensator_state_enum
{
	Ctrler_first,
	Ctrler_SS,
	Ctrler_grid_REL,
	Ctrler_active
};

struct Control_struct
{
	struct PI_struct PI_Im;
	struct PI_struct PI_Is;
	struct PR_struct PR_Io;
	struct PR_struct PR_Iz;
	float U_dc_filter;
	float U_dc_ref;
	float T_filtr_U_dc;
	float Ts;
	float ERR;
	float RDY;

	float U_lim;
	float I_lim;
	float L_comp;
	float Cdc;

	struct CIC_struct CIC_load_q_a, CIC_load_q_b, CIC_load_q_c;

	enum Control_state_enum state, state_last;
};

extern struct Control_struct Ctrl;

void Compensator_calc(float enable);

#endif /* Compensator_H_ */