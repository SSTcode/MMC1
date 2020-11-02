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

	struct PI_struct PI_ov;
	struct PI_struct PI_zv;
	struct PI_struct PI_sv;
	struct PI_struct PI_mv;

	struct PI_struct PI_Iod;
	struct PI_struct PI_Ioq;
	struct PI_struct PI_oi_d;
	struct PI_struct PI_oi_q;
	struct PI_struct PI_zi_d;
	struct PI_struct PI_zi_q;
	struct PI_struct PI_Izd;
	struct PI_struct PI_Izq;
	struct PI_struct PI_si;
	struct PI_struct PI_mi;

	float duty_modxy[6];

	struct PR_struct PR_Io;
	struct PR_struct PR_Iz;

	float Mm;
	float Ms;
	struct transformation_struct Mo;
	struct transformation_struct Mz;

	struct transformation_struct Io_struct;
	struct transformation_struct Io_ref;
	struct transformation_struct Iz_struct;
	struct transformation_struct Iz_ref;
	struct transformation_struct Io_ref_struct;
	struct transformation_struct Iz_ref_struct;

	struct transformation_struct Voref_struct;
	struct transformation_struct Vzref_struct;

	float n_cell;

	float Ts;
	float C;
	float ERR;
	float RDY;
	float U_dc;
	float Vdc;
	float Vx;

	float Uc_xy_filter[5];
	float Uc_xy_filter2[5];
	float Uc_xy_filter_coeff;

	float Ixy[6];
	float Io[3];
	float Iz[3];
	float Is;
	float Im;

	float Vxy[6];
	float Vo[3];
	float Vz[3];
	float Vs;
	float Vm;

	float I_xy_filter[5];
	float I_xy_filter2[5];
	float I_xy_filter_coeff;

	float izref[3];
	float ioref[3];
	float isref;
	float Vmrefv;
	float Vmrefi;
	float Vmref;
	
	float Voref[3];
	float Vzref[3];
	float Vsref;

	float is_sign;
	float Is_est;

	float Io_refT[3];
	float Iz_refT[3];
	
	float err_oi[3];
	float err_zi[3];
	float err_si;

	float err_ov[3];
	float err_zv[3];
	float err_sv;
	float err_mv;
	float Vc_ref;
	float Exy_ref;


	float Exy[6];
	float Eo[3];
	float Ez[3];
	float Es;
	float Em;

	float Mxy[6];

	float Pxy[6];
	float Po[3];
	float Pz[3];
	float Ps;
	float Pm;



	struct CIC_struct CIC_Ux_dc_p, CIC_Ux_dc_n, CIC_Ixy[6];

	struct DEC_struct xy2Dec;

	enum Control_state_enum state, state_last;
};

extern struct Control_struct Ctrl;

void Control_calc(float enable);

#endif /* Compensator_H_ */