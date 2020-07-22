#include "stdafx.h"
#include "Control.h"


struct Control_struct Ctrl;

void Compensator_calc(float enable)
{


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
			if (Meas.U_dc > 100.0f) 
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




			break;
		}
		default:
		{
			break;
		}
		}
	}
}