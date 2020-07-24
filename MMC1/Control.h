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

	float Mm;
	float Ms;
	struct transformation_struct Mo;
	struct transformation_struct Mz;

	float Im;
	float Is;
	struct transformation_struct Io;
	struct transformation_struct Iz;

	float Ts;
	float ERR;
	float RDY;
	float U_dc;



	struct CIC_struct CIC_Ux_dc_p, CIC_Ux_dc_n;

	enum Control_state_enum state, state_last;
};

extern struct Control_struct Ctrl;

void Control_calc(float enable);

#endif /* Compensator_H_ */