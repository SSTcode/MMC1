#include "stdafx.h"
#include "Control.h"


struct Control_struct Ctrl;

void Control_calc(float enable)
{	
	int i;
	for (i = 0; i < 6; i++) {

		Ctrl.Uc_xy_filter[i] += Ctrl.Uc_xy_filter_coeff * (Meas.Uc_pn[i] - Ctrl.Uc_xy_filter[i]);
		//Ctrl.U_dc_filter2 += Ctrl.U_dc_filter_coeff * (Ctrl.U_dc_filter - Ctrl.U_dc_filter2);
		
		Ctrl.Exy[i] = Ctrl.Uc_xy_filter[i] * Ctrl.Uc_xy_filter[i] * Ctrl.C * 0.5f;
		
		Ctrl.I_xy_filter[i] += Ctrl.I_xy_filter_coeff * (Meas.Ixy[i] - Ctrl.I_xy_filter[i]);
		//Ctrl.U_dc_filter2 += Ctrl.U_dc_filter_coeff * (Ctrl.U_dc_filter - Ctrl.U_dc_filter2);

		Ctrl.Ixy[i] = Ctrl.I_xy_filter[i];

		Ctrl.Pxy[i] = Ctrl.Mxy[i] * Ctrl.Ixy[i];
	}

	if (Ctrl.Vdc < 1.0) Ctrl.Vdc = 1.0f;
	Ctrl.Vx = Ctrl.Vdc * 0.5f;
	if (Ctrl.Vc_ref < Ctrl.Vdc / 4.0f * 1.2f) Ctrl.Vc_ref = Ctrl.Vdc / 4.0f * 1.2f;
	Ctrl.Exy_ref = Ctrl.Vc_ref * Ctrl.Vc_ref * Ctrl.C * 0.5f;
	float Io_max = 8.0f;
	float Umax_i = Ctrl.Vc_ref * Ctrl.n_cell;
	float Umax_v = Ctrl.Vx * Io_max;
	float Umin_i = -Umax_i;
	float Umin_v = -Umax_v;



	static float decimation = 0;
	if (++decimation > 1.0f)
	{
		decimation = 0;
		//CIC2_filter(Ctrl.CIC_Ux_dc_p, Meas.Uc_dc.p);
		//CIC2_filter(Ctrl.CIC_Ux_dc_n, Meas.Ux_dc.n);
	}


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
			
			xy2Dec(&Ctrl.xy2Dec, Ctrl.Exy);
	
			Ctrl.err_ov[0] = Ctrl.xy2Dec.o[0];//Ctrl.Eo[0];
			Ctrl.err_ov[1] = Ctrl.xy2Dec.o[1];//Ctrl.Eo[1];
			Ctrl.err_ov[2] = Ctrl.xy2Dec.o[2];//Ctrl.Eo[2];
							
			Ctrl.err_zv[0] = Ctrl.xy2Dec.z[0];// Ctrl.Ez[0];
			Ctrl.err_zv[1] = Ctrl.xy2Dec.z[1]; //Ctrl.Ez[1];
			Ctrl.err_zv[2] = Ctrl.xy2Dec.z[2]; //Ctrl.Ez[2];

			Ctrl.err_sv = Ctrl.xy2Dec.s;
			Ctrl.err_mv = Ctrl.Exy_ref - Ctrl.xy2Dec.m;

			PI_tustin(&Ctrl.PI_sv, Ctrl.err_sv);
			PI_tustin(&Ctrl.PI_mv, Ctrl.err_mv);
			PI_tustin_3ph(&Ctrl.PI_ov, Ctrl.err_ov);
			PI_tustin_3ph(&Ctrl.PI_zv, Ctrl.err_zv);

			Ctrl.izref[0] = Ctrl.PI_ov.out_3ph[0] / Ctrl.Vx;
			Ctrl.izref[1] = Ctrl.PI_ov.out_3ph[1] / Ctrl.Vx;
			Ctrl.izref[2] = Ctrl.PI_ov.out_3ph[2] / Ctrl.Vx;

			Ctrl.ioref[0] = Ctrl.PI_zv.out_3ph[0] / Ctrl.Vx;
			Ctrl.ioref[1] = Ctrl.PI_zv.out_3ph[1] / Ctrl.Vx;
			Ctrl.ioref[2] = Ctrl.PI_zv.out_3ph[2] / Ctrl.Vx;

			Ctrl.isref = Ctrl.PI_mv.out / Ctrl.Vx;

			if (Ctrl.isref > 0)Ctrl.is_sign = 1.0;
			else Ctrl.is_sign = -1.0;

			Ctrl.Vmrefv = -1.0 * Ctrl.PI_sv.out/Ctrl.Is_est * Ctrl.is_sign;

			xy2Dec(&Ctrl.xy2Dec, Ctrl.Ixy);
			Ctrl.Io[0] = Ctrl.xy2Dec.o[0];
			Ctrl.Io[1] = Ctrl.xy2Dec.o[1];
			Ctrl.Io[2] = Ctrl.xy2Dec.o[2];

			Ctrl.Iz[0] = Ctrl.xy2Dec.z[0];
			Ctrl.Iz[1] = Ctrl.xy2Dec.z[1];
			Ctrl.Iz[2] = Ctrl.xy2Dec.z[2];

			Ctrl.Is = Ctrl.xy2Dec.s;
			Ctrl.Im = Ctrl.xy2Dec.m;

			//Control de Io 
			Ctrl.Io_refT[0] = Meas.Io_ref.a + Ctrl.ioref[0];
			Ctrl.Io_refT[1] = Meas.Io_ref.b + Ctrl.ioref[1];
			Ctrl.Io_refT[2] = Meas.Io_ref.c + Ctrl.ioref[2];
			
			Ctrl.err_oi[0] = Ctrl.Io_refT[0] - Ctrl.Io[0];
			Ctrl.err_oi[1] = Ctrl.Io_refT[1] - Ctrl.Io[1];
			Ctrl.err_oi[2] = Ctrl.Io_refT[2] - Ctrl.Io[2];


			//Control de Iz
			Ctrl.Iz_refT[0] = Meas.Iz_ref.a + Ctrl.izref[0];
			Ctrl.Iz_refT[1] = Meas.Iz_ref.b + Ctrl.izref[1];
			Ctrl.Iz_refT[2] = Meas.Iz_ref.c + Ctrl.izref[2];
		   
			Ctrl.err_zi[0] = Ctrl.Iz_refT[0] - Ctrl.Iz[0];
			Ctrl.err_zi[1] = Ctrl.Iz_refT[1] - Ctrl.Iz[1];
			Ctrl.err_zi[2] = Ctrl.Iz_refT[2] - Ctrl.Iz[2];

			//Control de Is	
			Ctrl.err_si = Ctrl.isref - Ctrl.Is;

			//Control de Im	
			Ctrl.Vmrefi = Ctrl.Vmrefv;// +Vm_ripple;


			PI_tustin(&Ctrl.PI_si, Ctrl.err_si);
			PI_tustin_3ph(&Ctrl.PI_oi, Ctrl.err_oi);
			PI_tustin_3ph(&Ctrl.PI_zi, Ctrl.err_zi);

			Ctrl.Voref[0] = Ctrl.PI_oi.out_3ph[0];
			Ctrl.Voref[1] = Ctrl.PI_oi.out_3ph[1];
			Ctrl.Voref[2] = Ctrl.PI_oi.out_3ph[2];

			Ctrl.Vzref[0] = Ctrl.PI_zi.out_3ph[0];
			Ctrl.Vzref[1] = Ctrl.PI_zi.out_3ph[1];
			Ctrl.Vzref[2] = Ctrl.PI_zi.out_3ph[2];

			Ctrl.Vsref = Ctrl.PI_si.out;

			Dec2xy(&Ctrl.xy2Dec, Ctrl.Voref, Ctrl.Vsref, Ctrl.Vzref, Ctrl.Vmrefi);
			Ctrl.Vxy[0] = Ctrl.xy2Dec.xy[0];
			Ctrl.Vxy[1] = Ctrl.xy2Dec.xy[1];
			Ctrl.Vxy[2] = Ctrl.xy2Dec.xy[2];
			Ctrl.Vxy[3] = Ctrl.xy2Dec.xy[3];
			Ctrl.Vxy[4] = Ctrl.xy2Dec.xy[4];
			Ctrl.Vxy[5] = Ctrl.xy2Dec.xy[5];


			for (i = 0; i < 3; i++) {
				Ctrl.Mxy[i] = Ctrl.Vx - Meas.Vgrid[i] - Ctrl.Vxy[i];
				if (Ctrl.Mxy[i] > Umax_i)
					Ctrl.Mxy[i] = Umax_i;
				else if (Ctrl.Mxy[i] < Umin_i)
					Ctrl.Mxy[i] = Umin_i;
				else
					Ctrl.Mxy[i] = Ctrl.Vx - Meas.Vgrid[i] - Ctrl.Vxy[i];

				//Mxy[i]=Vx-Vabc[i]-Vxy[i];

			}
			for (i = 3; i < 6; i++) {
				Ctrl.Mxy[i] = -Ctrl.Vx - Meas.Vgrid[i - 3] - Ctrl.Vxy[i];
				if (Ctrl.Mxy[i] > Umax_i)
					Ctrl.Mxy[i] = Umax_i;
				else if (Ctrl.Mxy[i] < Umin_i)
					Ctrl.Mxy[i] = Umin_i;
				else
					Ctrl.Mxy[i] = -Ctrl.Vx - Meas.Vgrid[i - 3] - Ctrl.Vxy[i];
				//Mxy[i]=-Vx-Vabc[i-3]-Vxy[i]; 
			}
		

		Ctrl.duty_modxy[0] = Ctrl.Mxy[0] / Ctrl.Vc_ref / Ctrl.n_cell;
		Ctrl.duty_modxy[1] = Ctrl.Mxy[1] / Ctrl.Vc_ref / Ctrl.n_cell;
		Ctrl.duty_modxy[2] = Ctrl.Mxy[2] / Ctrl.Vc_ref / Ctrl.n_cell;
		Ctrl.duty_modxy[3] = Ctrl.Mxy[3] / Ctrl.Vc_ref / Ctrl.n_cell;
		Ctrl.duty_modxy[4] = Ctrl.Mxy[4] / Ctrl.Vc_ref / Ctrl.n_cell;
		Ctrl.duty_modxy[5] = Ctrl.Mxy[5] / Ctrl.Vc_ref / Ctrl.n_cell;


			//Permanent Regimen
			Ctrl.Ms = 0.249f;;
			Ctrl.Mm  = 0.2499650000f;
			Ctrl.Mo.a= 0.2536f* MATH_1_SQRT2*sin(PLL.theta_1-172.86f*3.14f/180.0f);
			Ctrl.Mo.b= 0.2536f * MATH_1_SQRT2 * sin(PLL.theta_1 - 172.86f * 3.14f / 180.0f-MATH_2PI_3);
			Ctrl.Mo.c= 0.2536f * MATH_1_SQRT2 * sin(PLL.theta_1 - 172.86f * 3.14f / 180.0f+ MATH_2PI_3);
			Ctrl.Mz.a= 0.007850995160f* MATH_1_SQRT2 * sin(2*PLL.theta_1 -90.91227607 * 3.14f / 180.0f);
			Ctrl.Mz.b= 0.007850995160f * MATH_1_SQRT2 * sin(2*PLL.theta_1 - 90.91227607 * 3.14f / 180.0f- MATH_2PI_3);
			Ctrl.Mz.c= 0.007850995160f * MATH_1_SQRT2 * sin(2*PLL.theta_1 - 90.91227607 * 3.14f / 180.0f+ MATH_2PI_3);

			
			break;
		}
		default:
		{
			break;
		}
		}
	}
}