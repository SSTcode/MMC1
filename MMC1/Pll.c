#include "stdafx.h"
#include "PLL.h"

struct PLL_struct PLL;

void PLL_calc(float enable)
{
	static struct transformation_struct signal_pll = { 0 };
	signal_pll.a = Meas.Io_ref.a;
	signal_pll.b = Meas.Io_ref.b;
	signal_pll.c = Meas.Io_ref.c;

	abc_abg(signal_pll);
	abg_dqz(signal_pll, PLL.theta_1);
	Meas.theta = atan2f(signal_pll.beta, signal_pll.alfa);
	static float theta_last;




	if (!enable && !PLL.ERR)
	{
		PLL.state = omega_preinit;
		PLL.state_last = omega_preinit;
		PLL.RDY = 0;
	}
	else
	{
		switch (PLL.state)
		{
		case omega_preinit:
		{
			PLL.state++;
			break;
		}
		case omega_init:
		{
			static float counter_calka;
			static float calka;
			if (PLL.state_last != PLL.state)
			{
				PLL.state_last = PLL.state;

				calka = 0.0f;
				counter_calka = 0;
			}

			float error = Meas.theta - theta_last;
			calka += error - (float)((int32)(error / MATH_PI)) * MATH_2PI;

			Uint16 counter_val = 1.0f / PLL.Ts * 0.01f;
			counter_calka++;
			if (counter_calka >= counter_val)
			{
				float omega_est = 1.0f / PLL.Ts / (float)counter_val * calka;

				PLL.theta_1 = Meas.theta;
				PLL.w_filter1 = omega_est;
				PLL.w_filter2 = omega_est;
				PLL.PI.integrator = omega_est;

				PLL.SOGI_alf.x = signal_pll.alfa;
				PLL.SOGI_bet.x = signal_pll.beta;

				if (omega_est > 0)
				{
					PLL.SOGI_alf.qx = signal_pll.beta;
					PLL.SOGI_bet.qx = -signal_pll.alfa;
				}
				else
				{
					PLL.SOGI_alf.qx = -signal_pll.beta;
					PLL.SOGI_bet.qx = signal_pll.alfa;
				}

				PLL.state++;
			}
			break;
		}
		case PLL_check:
		{
			static float synch_ok_counter;
			static float synch_counter;
			static float counter_avg;

			static float omega_avg_calka;
			static float omega_avg_last;
			static float omega_max_run;
			static float omega_min_run;

			if (PLL.state_last != PLL.state)
			{
				PLL.state_last = PLL.state;

				counter_avg = 0;
				synch_counter = 0;
				synch_ok_counter = 0;

				omega_avg_calka = 0;
				omega_avg_last = PLL.w_filter2;
				omega_max_run = PLL.w_filter2;
				omega_min_run = PLL.w_filter2;
			}

			omega_avg_calka += PLL.w_filter2;
			omega_max_run = fmaxf(omega_max_run, PLL.w_filter2);
			omega_min_run = fmaxf(omega_min_run, PLL.w_filter2);

			// verificamos cada 1 período para eliminar el impacto de la oscilación en el valor promedio
			counter_avg++;
			Uint16 counter_val = 1.0f / PLL.Ts * 0.02f;
			if (counter_avg >= counter_val) {
				float omega_avg = omega_avg_calka / (float)counter_val;

				if ((fabs(omega_avg - omega_avg_last) <= 0.05f) &&
					(fabs(omega_avg - omega_max_run) <= 0.2f) &&
					(fabs(omega_avg - omega_min_run) <= 0.2f))
				{
					synch_ok_counter++;
				}
				else synch_ok_counter = 0;

				omega_avg_last = omega_avg;
				omega_avg_calka = 0;
				omega_max_run = PLL.w_filter2;
				omega_min_run = PLL.w_filter2;

				counter_avg = 0;
				synch_counter++;
			}

			if ((synch_counter == 40) && (PLL.RDY == 0)) PLL.ERR = 1;
			if (synch_ok_counter >= 5)
			{
				PLL.RDY = 1;
				PLL.state++;
			}
		}
		case PLL_active:
		{
			static struct transformation_struct x_pos;
			if (PLL.state_last != PLL.state)
			{
				PLL.state_last = PLL.state;
				x_pos = (const struct transformation_struct){ 0 };
			}

			//SOGI alf/bet
			PLL.w = fabs(PLL.PI.out);
			SOGI_calc(&PLL.SOGI_alf, signal_pll.alfa, PLL.w);
			SOGI_calc(&PLL.SOGI_bet, signal_pll.beta, PLL.w);

			// Determinar el componente compatible de la señal en alf / bet
			if (PLL.PI.out > 0.0f)
			{
				x_pos.alfa = 0.5f * (PLL.SOGI_alf.x - PLL.SOGI_bet.qx);
				x_pos.beta = 0.5f * (PLL.SOGI_bet.x + PLL.SOGI_alf.qx);
			}
			else
			{
				x_pos.alfa = 0.5f * (PLL.SOGI_alf.x + PLL.SOGI_bet.qx);
				x_pos.beta = 0.5f * (PLL.SOGI_bet.x - PLL.SOGI_alf.qx);
			}

			// Conversión a dq de un componente compatible de señal alf / bet
			abg_dqz(x_pos, PLL.theta_1);

			//PI
			float Umod_pos_PLL = sqrtf(x_pos.alfa * x_pos.alfa + x_pos.beta * x_pos.beta); //Amplitude
			float error_PLL = x_pos.q / fmaxf(Umod_pos_PLL, 1.0f);
			PI_antiwindup_fast(&PLL.PI, error_PLL);

			// Calcula el ángulo // Comprueba la dirección de giro
			PLL.theta_1 += PLL.Ts * PLL.PI.out;
			PLL.theta_1 -= (float)((int32)(PLL.theta_1 / MATH_PI)) * MATH_2PI;
			PLL.theta_2 = PLL.theta_1 - MATH_2PI_3;
			PLL.theta_2 -= (float)((int32)(PLL.theta_2 / MATH_PI)) * MATH_2PI;
			PLL.theta_3 = PLL.theta_1 + MATH_2PI_3;
			PLL.theta_3 -= (float)((int32)(PLL.theta_3 / MATH_PI)) * MATH_2PI;



			float T_filter = 0.01f;
			// Omega filtrado, filtro de paso bajo 1 orden
			PLL.w_filter1 += PLL.Ts / T_filter * (PLL.PI.out - PLL.w_filter1);
			// Omega filtrado, filtro de paso bajo 2 orden
			PLL.w_filter2 += PLL.Ts / T_filter * (PLL.w_filter1 - PLL.w_filter2);
			
			PLL.f = PLL.w * MATH_1_2PI;
			PLL.f_filter1 = PLL.w_filter1 * MATH_1_2PI;
			PLL.f_filter2 = PLL.w_filter2 * MATH_1_2PI;

			//if (PLL.theta_4 >= MATH_2PI) PLL.theta_4 = PLL.theta_4 - MATH_2PI;
			//else PLL.theta_4 = PLL.theta_4;
			PLL.theta_4 +=2.0f* PLL.w * PLL.Ts;
			PLL.theta_4 -= (float)((int32)(PLL.theta_4 / MATH_PI)) * MATH_2PI;
			PLL.theta_5 = PLL.theta_4 - MATH_2PI_3;
			PLL.theta_5 -= (float)((int32)(PLL.theta_5 / MATH_PI)) * MATH_2PI;
			PLL.theta_6 = PLL.theta_4 + MATH_2PI_3;
			PLL.theta_6 -= (float)((int32)(PLL.theta_6 / MATH_PI)) * MATH_2PI;
			

			break;
		}
		default:
			PLL.ERR = 1;
			break;
		}
	}
	theta_last = Meas.theta;
}