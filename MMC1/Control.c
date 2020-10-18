#include "stdafx.h"
#include "Control.h"


struct Control_struct Ctrl;

void Control_calc(float enable)
{	

	int i;
	for (i = 0; i < 6; i++) {

		//Ctrl.Uc_xy_filter[i] += Ctrl.Uc_xy_filter_coeff * (Meas.Uc_pn[i] - Ctrl.Uc_xy_filter[i]);
		//Ctrl.U_dc_filter2 += Ctrl.U_dc_filter_coeff * (Ctrl.U_dc_filter - Ctrl.U_dc_filter2);
		
		//Ctrl.Exy[i] = Ctrl.Uc_xy_filter[i] * Ctrl.Uc_xy_filter[i] * Ctrl.C * 0.5f;
		
	//static float decimation = 0;
	//if (++decimation > 1.0f)
	//{
	//	decimation = 0;
	//	CIC2_filter(Ctrl.CIC_Ixy[i], Meas.Ixy[i]);
	//	//CIC2_filter(Ctrl.CIC_Ux_dc_n, Meas.Ux_dc.n);
	//}
		//Ctrl.I_xy_filter[i] += Ctrl.I_xy_filter_coeff * (Meas.Ixy[i] - Ctrl.I_xy_filter[i]);
		//Ctrl.U_dc_filter2 += Ctrl.U_dc_filter_coeff * (Ctrl.U_dc_filter - Ctrl.U_dc_filter2);

		Ctrl.Ixy[i] = Meas.Ixy[i];

		//Ctrl.Pxy[i] = Ctrl.Mxy[i] * Ctrl.Ixy[i];
	}

	//if (Ctrl.Vdc < 1.0) Ctrl.Vdc = 1.0f;
	//Ctrl.Vx = Ctrl.Vdc * 0.5f;
	//if (Ctrl.Vc_ref < Ctrl.Vdc / 4.0f * 1.2f) Ctrl.Vc_ref = Ctrl.Vdc / 4.0f * 1.2f;
	//Ctrl.Exy_ref = Ctrl.Vc_ref * Ctrl.Vc_ref * Ctrl.C * 0.5f;
	//float Io_max = 8.0f;
	//float Umax_i = Ctrl.Vc_ref * Ctrl.n_cell;
	//float Umax_v = Ctrl.Vx * Io_max;
	//float Umin_i = -Umax_i;
	//float Umin_v = -Umax_v;





	if (!enable || Ctrl.ERR)
	{
		Ctrl.state = Ctrler_first;
		Ctrl.state_last = Ctrler_first;
	}
	else
	{
	
		switch (Ctrl.state)
		{
		case Ctrler_first:
		{
			Ctrl.state++;
			break;
		}
		case Ctrler_SS:
		{
			
			static float counter_ss;
			if (Ctrl.state_last != Ctrl.state)
			{
				counter_ss = 0;
				Ctrl.state_last = Ctrl.state;
			}
			if (++counter_ss > 100)
			{
				Ctrl.ERR = 1.0f;
			}
			//Ctrl.U_dc = Ctrl.CIC_Ux_dc_p.out + Ctrl.CIC_Ux_dc_n.out;
			if (Ctrl.U_dc > 100.0f)
			
				Ctrl.err_mv = 100;
				Ctrl.state++;
			
			break;
		}
		case Ctrler_grid_REL:
		{
			static float counter_ss;
			Ctrl.err_sv = counter_ss;
			if (Ctrl.state_last != Ctrl.state)
			{
				counter_ss = 0;
				Ctrl.state_last = Ctrl.state;
			}
			if (++counter_ss > 10) Ctrl.state++;
			Ctrl.err_sv = counter_ss;
			break;
		}
		case Ctrler_active:
		{
			
			//xy2Dec(&Ctrl.xy2Dec, Ctrl.Exy);
			//
			//Ctrl.err_ov[0] = Ctrl.xy2Dec.o[0];//Ctrl.Eo[0];
			//Ctrl.err_ov[1] = Ctrl.xy2Dec.o[1];//Ctrl.Eo[1];
			//Ctrl.err_ov[2] = Ctrl.xy2Dec.o[2];//Ctrl.Eo[2];
			//				
			//Ctrl.err_zv[0] = Ctrl.xy2Dec.z[0];// Ctrl.Ez[0];
			//Ctrl.err_zv[1] = Ctrl.xy2Dec.z[1]; //Ctrl.Ez[1];
			//Ctrl.err_zv[2] = Ctrl.xy2Dec.z[2]; //Ctrl.Ez[2];
			//
			//Ctrl.err_sv = Ctrl.xy2Dec.s;
			//Ctrl.err_mv = Ctrl.Exy_ref - Ctrl.xy2Dec.m;
			//
			//PI_tustin(&Ctrl.PI_sv, Ctrl.err_sv);
			//PI_tustin(&Ctrl.PI_mv, Ctrl.err_mv);
			//PI_tustin_3ph(&Ctrl.PI_ov, Ctrl.err_ov);
			//PI_tustin_3ph(&Ctrl.PI_zv, Ctrl.err_zv);

			//Ctrl.izref[0] = Ctrl.PI_ov.out_3ph[0] / Ctrl.Vx;
			//Ctrl.izref[1] = Ctrl.PI_ov.out_3ph[1] / Ctrl.Vx;
			//Ctrl.izref[2] = Ctrl.PI_ov.out_3ph[2] / Ctrl.Vx;
			//
			//Ctrl.ioref[0] = Ctrl.PI_zv.out_3ph[0] / Ctrl.Vx;
			//Ctrl.ioref[1] = Ctrl.PI_zv.out_3ph[1] / Ctrl.Vx;
			//Ctrl.ioref[2] = Ctrl.PI_zv.out_3ph[2] / Ctrl.Vx;
			//
			//Ctrl.isref = Ctrl.PI_mv.out / Ctrl.Vx;
			//
			//if (Ctrl.isref > 0)Ctrl.is_sign = 1.0;
			//else Ctrl.is_sign = -1.0;
			//
			//Ctrl.Vmrefv = 0.0;// -1.0 * Ctrl.PI_sv.out / Ctrl.Is_est * Ctrl.is_sign;
			//
			xy2Dec(&Ctrl.xy2Dec, Ctrl.Ixy);
			//
			Ctrl.Iz_struct.a = Ctrl.xy2Dec.z[0];
			Ctrl.Iz_struct.b = Ctrl.xy2Dec.z[1];
			Ctrl.Iz_struct.c = Ctrl.xy2Dec.z[2];
			Ctrl.Io_struct.a = Ctrl.xy2Dec.o[0];
			Ctrl.Io_struct.b = Ctrl.xy2Dec.o[1];
			Ctrl.Io_struct.c = Ctrl.xy2Dec.o[2];
			Ctrl.Is			 = Ctrl.xy2Dec.s;
			Ctrl.Im			 = Ctrl.xy2Dec.m;
			//
			//abc_abg(Ctrl.Io_struct);
			//abg_dqz(Ctrl.Io_struct, PLL.theta_1);
			//
			////abc_dq_pos(Ctrl.Io_struct, PLL.theta_1);
			//

			//
			//abc_dq_pos(Ctrl.Iz_struct, -PLL.theta_4);
			//


			//Trasformation
			//static struct transformation_struct Io = { 0 };


		

			//Ctrl.Io_ref_struct.a = Meas.Io_ref.a;
			//Ctrl.Io_ref_struct.b = Meas.Io_ref.b;
			//Ctrl.Io_ref_struct.c = Meas.Io_ref.c;
			//abc_dq_pos(Ctrl.Io_ref_struct, PLL.theta_1);
			//
			//Ctrl.Iz_ref_struct.a = Meas.Iz_ref.a;
			//Ctrl.Iz_ref_struct.b = Meas.Iz_ref.b;
			//Ctrl.Iz_ref_struct.c = Meas.Iz_ref.c;
			//abc_dq_neg(Ctrl.Iz_ref_struct, PLL.theta_4);
			//
			////Control de Io
			//Ctrl.err_oi[0] = Ctrl.Io_ref_struct.d - Ctrl.Io_struct.d;
			//Ctrl.err_oi[1] = Ctrl.Io_ref_struct.q - Ctrl.Io_struct.q;
			//
			////Control de Iz
			//Ctrl.err_zi[0] = Ctrl.Iz_ref_struct.d - Ctrl.Iz_struct.d;
			//Ctrl.err_zi[1] = Ctrl.Iz_ref_struct.q - Ctrl.Iz_struct.q;
			//
			////Control de Is	
			//Ctrl.err_si = Meas.Is_ref - Ctrl.Is;
			//
			////Control de Im	
			//Ctrl.Vmrefi = Ctrl.Vmrefv;// +Vm_ripple;
			//
			//
			//PI_tustin(&Ctrl.PI_oi_d, Ctrl.err_oi[0]);
			//PI_tustin(&Ctrl.PI_oi_q, Ctrl.err_oi[1]);
			//PI_tustin(&Ctrl.PI_zi_d, Ctrl.err_zi[0]);
			//PI_tustin(&Ctrl.PI_zi_q, Ctrl.err_zi[1]);
			//PI_tustin(&Ctrl.PI_Is, Ctrl.err_si);
			//
			//
			//Ctrl.Voref_struct.d = Ctrl.PI_oi_d.out;
			//Ctrl.Voref_struct.q = Ctrl.PI_oi_q.out;
			//
			//Ctrl.Vzref_struct.d = Ctrl.PI_zi_d.out;
			//Ctrl.Vzref_struct.q = Ctrl.PI_zi_q.out;
			//
			//Ctrl.Vsref = Ctrl.PI_Is.out;
			//
			//
			////Trasformation
			//dq_abc_pos(Ctrl.Voref_struct, PLL.theta_1);
			//
			//dq_abc_neg(Ctrl.Vzref_struct, PLL.theta_3);

			//Dec2xy(Ctrl.xy2Dec, Ctrl.Voref_struct, Ctrl.Vsref, Ctrl.Vzref_struct, Ctrl.Vmrefi);
			//Ctrl.Vxy[0] = Ctrl.xy2Dec.pa;
			//Ctrl.Vxy[1] = Ctrl.xy2Dec.pb;
			//Ctrl.Vxy[2] = Ctrl.xy2Dec.pc;
			//Ctrl.Vxy[3] = Ctrl.xy2Dec.na;
			//Ctrl.Vxy[4] = Ctrl.xy2Dec.nb;
			//Ctrl.Vxy[5] = Ctrl.xy2Dec.nc;

			//Amplitudes Permanent Regimen
			//static float theta_1 = 0;
			//theta_1+= PLL.Ts * MATH_2PI * 50;
			//if (theta_1 >= MATH_2PI) theta_1 = theta_1 - MATH_2PI;
			//else theta_1 = theta_1;
			//Ctrl.Ms = 0.249f;
			//Ctrl.Mm  = 0.2499650000f;
			//Ctrl.Mo.a= 50.2536f * MATH_1_SQRT2 * sin(theta_1 - 172.86f * 3.14f / 180.0f);
			//Ctrl.Mo.b= 50.2536f * MATH_1_SQRT2 * sin(theta_1 - 172.86f * 3.14f / 180.0f - MATH_2PI_3);
			//Ctrl.Mo.c= 50.2536f * MATH_1_SQRT2 * sin(theta_1 - 172.86f * 3.14f / 180.0f + MATH_2PI_3);
			//Ctrl.Mz.a= 0.007850995160f * MATH_1_SQRT2 * sin(2*theta_1 - 90.91227607 * 3.14f / 180.0f);
			//Ctrl.Mz.b= 0.007850995160f * MATH_1_SQRT2 * sin(2*theta_1 - 90.91227607 * 3.14f / 180.0f - MATH_2PI_3);
			//Ctrl.Mz.c= 0.007850995160f * MATH_1_SQRT2 * sin(2*theta_1 - 90.91227607 * 3.14f / 180.0f + MATH_2PI_3);

			//Dec2xy(Ctrl.xy2Dec, Ctrl.Voref_struct, Meas.Is_ref, Meas.Iz_ref, Ctrl.Mm);
			//Ctrl.Vxy[0] = Ctrl.xy2Dec.pa;
			//Ctrl.Vxy[1] = Ctrl.xy2Dec.pb;
			//Ctrl.Vxy[2] = Ctrl.xy2Dec.pc;
			//Ctrl.Vxy[3] = Ctrl.xy2Dec.na;
			//Ctrl.Vxy[4] = Ctrl.xy2Dec.nb;
			//Ctrl.Vxy[5] = Ctrl.xy2Dec.nc;

			

			//for (i = 0; i < 3; i++) {
			//	Ctrl.Mxy[i] = Ctrl.Vx - Meas.Vgrid[i] - Ctrl.Vxy[i];
			//	if (Ctrl.Mxy[i] > Umax_i)
			//		Ctrl.Mxy[i] = Umax_i;
			//	else if (Ctrl.Mxy[i] < Umin_i)
			//		Ctrl.Mxy[i] = Umin_i;
			//	else
			//		Ctrl.Mxy[i] = Ctrl.Vx - Meas.Vgrid[i] - Ctrl.Vxy[i];
			//
			//	//Mxy[i]=Vx-Vabc[i]-Vxy[i];
			//
			//}
			//for (i = 3; i < 6; i++) {
			//	Ctrl.Mxy[i] = -Ctrl.Vx - Meas.Vgrid[i - 3] - Ctrl.Vxy[i];
			//	if (Ctrl.Mxy[i] > Umax_i)
			//		Ctrl.Mxy[i] = Umax_i;
			//	else if (Ctrl.Mxy[i] < Umin_i)
			//		Ctrl.Mxy[i] = Umin_i;
			//	else
			//		Ctrl.Mxy[i] = -Ctrl.Vx - Meas.Vgrid[i - 3] -* Ctrl.Vxy[i];
			//	//Mxy[i]=-Vx-Vabc[i-3]-Vxy[i]; 
			//}
			//
			//
			//Ctrl.duty_modxy[0] = Ctrl.Mxy[0] / Ctrl.Vc_ref / Ctrl.n_cell;
			//Ctrl.duty_modxy[1] = Ctrl.Mxy[1] / Ctrl.Vc_ref / Ctrl.n_cell;
			//Ctrl.duty_modxy[2] = Ctrl.Mxy[2] / Ctrl.Vc_ref / Ctrl.n_cell;
			//Ctrl.duty_modxy[3] = Ctrl.Mxy[3] / Ctrl.Vc_ref / Ctrl.n_cell;
			//Ctrl.duty_modxy[4] = Ctrl.Mxy[4] / Ctrl.Vc_ref / Ctrl.n_cell;
			//Ctrl.duty_modxy[5] = Ctrl.Mxy[5] / Ctrl.Vc_ref / Ctrl.n_cell;



			





			break;
		}
		default:
		{
			break;
		}
		}
	}
}