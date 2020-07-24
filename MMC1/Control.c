#include "stdafx.h"
#include "Control.h"


struct Control_struct Ctrl;

void Control_calc(float enable)
{
	
	
	Ctrl.Im = (Meas.Ixy_p.a + Meas.Ixy_p.b + Meas.Ixy_p.c + Meas.Ixy_n.a + Meas.Ixy_n.b + Meas.Ixy_n.c) * 0.166666667f;
	
	Ctrl.Is = ((Meas.Ixy_p.a + Meas.Ixy_p.b + Meas.Ixy_p.c) - (Meas.Ixy_n.a + Meas.Ixy_n.b + Meas.Ixy_n.c)) * 0.166666667f;
	
	Ctrl.Io.a = (2.0f * (Meas.Ixy_p.a + Meas.Ixy_n.a) - (Meas.Ixy_p.b + Meas.Ixy_n.b) - (Meas.Ixy_p.c + Meas.Ixy_n.c)) * 0.166666667f;
	Ctrl.Io.b = (-1.0f * (Meas.Ixy_p.a + Meas.Ixy_n.a) + 2.0f * (Meas.Ixy_p.b + Meas.Ixy_n.b) - (Meas.Ixy_p.c + Meas.Ixy_n.c)) * 0.166666667f;
	Ctrl.Io.c = (-1.0f * (Meas.Ixy_p.a + Meas.Ixy_n.a) - (Meas.Ixy_p.b + Meas.Ixy_n.b) + 2.0f * (Meas.Ixy_p.c + Meas.Ixy_n.c)) * 0.166666667f;
	
	Ctrl.Iz.a = (2.0f * (Meas.Ixy_p.a - Meas.Ixy_n.a) - (Meas.Ixy_p.b - Meas.Ixy_n.b) - (Meas.Ixy_p.c - Meas.Ixy_n.c)) * 0.166666667f;
	Ctrl.Iz.b = (-1.0f * (Meas.Ixy_p.a - Meas.Ixy_n.a) + 2.0f * (Meas.Ixy_p.b - Meas.Ixy_n.b) - (Meas.Ixy_p.c - Meas.Ixy_n.c)) * 0.166666667f;
	Ctrl.Iz.c = (-1.0f * (Meas.Ixy_p.a - Meas.Ixy_n.a) - (Meas.Ixy_p.b - Meas.Ixy_n.b) + 2.0f * (Meas.Ixy_p.c - Meas.Ixy_n.c)) * 0.166666667f;

	static float decimation = 0;
	if (++decimation > 1.0f)
	{
		decimation = 0;
		CIC2_filter(Ctrl.CIC_Ux_dc_p, Meas.Ux_dc.p);
		CIC2_filter(Ctrl.CIC_Ux_dc_n, Meas.Ux_dc.n);
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
			if (++counter_ss > 1000000)
			{
				Ctrl.ERR = 1.0f;
			}
			Ctrl.U_dc = Ctrl.CIC_Ux_dc_p.out + Ctrl.CIC_Ux_dc_n.out;
			if (Ctrl.U_dc > 100.0f)
				Ctrl.state++;
			
			break;
		}
		case Ctrler_grid_REL:
		{
			static float counter_ss;
			if (Ctrl.state_last != Ctrl.state)
			{
				counter_ss = 0;
				Ctrl.state_last = Ctrl.state;
			}
			if (++counter_ss > 1000) Ctrl.state++;
			break;
		}
		case Ctrler_active:
		{
			
			
	
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