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

			//Ctrl.Eo[0] = Ctrl.xy2Dec.o[0];// 0.166666667f * (2.0f * Ctrl.Exy[0] + 2.0f * Ctrl.Exy[3] - Ctrl.Exy[1] - Ctrl.Exy[4] - Ctrl.Exy[2] - Ctrl.Exy[5]);//1/6(2 ipa+2ina-ipb-inb-ipc-inc)
			//Ctrl.Eo[1] = Ctrl.xy2Dec.o[1];//0.166666667f * (-Ctrl.Exy[0] - Ctrl.Exy[3] + 2.0f * Ctrl.Exy[1] + 2.0f * Ctrl.Exy[4] - Ctrl.Exy[2] - Ctrl.Exy[5]);
			//Ctrl.Eo[2] = Ctrl.xy2Dec.o[2];//0.166666667f * (-Ctrl.Exy[0] - Ctrl.Exy[3] - Ctrl.Exy[1] - Ctrl.Exy[4] + 2.0f * Ctrl.Exy[2] + 2.0f * Ctrl.Exy[5]);
			//Ctrl.Es    = Ctrl.xy2Dec.s;   //0.166666667f * (Ctrl.Exy[0] - Ctrl.Exy[3] + Ctrl.Exy[1] - Ctrl.Exy[4] + Ctrl.Exy[2] - Ctrl.Exy[5]);
			//Ctrl.Ez[0] = Ctrl.xy2Dec.z[0];//0.166666667f * (2.0f * Ctrl.Exy[0] - 2.0f * Ctrl.Exy[3] - Ctrl.Exy[1] + Ctrl.Exy[4] - Ctrl.Exy[2] + Ctrl.Exy[5]);
			//Ctrl.Ez[1] = Ctrl.xy2Dec.z[1];//0.166666667f * (-Ctrl.Exy[0] + Ctrl.Exy[3] + 2.0f * Ctrl.Exy[1] - 2.0f * Ctrl.Exy[4] - Ctrl.Exy[2] + Ctrl.Exy[5]);
			//Ctrl.Ez[2] = Ctrl.xy2Dec.z[2];//0.166666667f * (-Ctrl.Exy[0] + Ctrl.Exy[3] - Ctrl.Exy[1] + Ctrl.Exy[4] + 2.0f * Ctrl.Exy[2] - 2.0f * Ctrl.Exy[5]);
			//Ctrl.Em    = Ctrl.xy2Dec.m;   // 0.166666667f * (Ctrl.Exy[0] + Ctrl.Exy[1] + Ctrl.Exy[2] + Ctrl.Exy[3] + Ctrl.Exy[4] + Ctrl.Exy[5]);
			
			
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


			//xy2Dec(&Ctrl.xy2Dec);









			
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