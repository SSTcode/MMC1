#pragma once
#ifndef Controllers_H_
#define Controllers_H_

extern const float MATH_PI;
extern const float MATH_2PI;
extern const float MATH_2PI_3;
extern const float MATH_PI_3;
extern const float MATH_1_2PI;
extern const float MATH_1_PI;
extern const float MATH_1_3;
extern const float MATH_1_SQRT3;
extern const float MATH_1_SQRT2;
extern const float MATH_SQRT2_3;
extern const float MATH_SQRT3_2;
extern const float MATH_SQRT2;
extern const float MATH_SQRT3;
extern const float MATH_2_3;

//Cascaded integrator–comb filter
#define CIC1_filter(CIC_struct, input)			                                                                                     \
{                                                                                                                                    \
register float div_osr = 1.0f / (float)CIC_struct.OSR;																				 \
register float limit = (int32)(4294967296.0 / 2.0 * div_osr);																		 \
CIC_struct.integrator[0] += (int32)fmaxf(fminf(input, limit), -limit);																 \
register float counter_temp = CIC_struct.counter + 1.0f;																			 \
counter_temp = (counter_temp - (float)((int32)(counter_temp*div_osr))*(float)CIC_struct.OSR);										 \
CIC_struct.counter = (int32)counter_temp;																							 \
register int32 div_memory_new = (int32)(counter_temp*20.0f*div_osr);																 \
if (CIC_struct.div_memory != div_memory_new)																						 \
{																																	 \
	CIC_struct.div_memory = div_memory_new;																							 \
	register int32 *subtractor = (int32 *)((int32(*)[2])&CIC_struct.subtractor + div_memory_new);									 \
	register int32 *decimator = (int32 *)&CIC_struct.decimator_memory + div_memory_new;												 \
	*subtractor = *decimator;																										 \
	*decimator = CIC_struct.integrator[0];																							 \
	CIC_struct.out = (float)(CIC_struct.integrator[0] - *subtractor) * div_osr;														 \
}																																	 \
}

//Cascaded integrator?comb filter
#define CIC2_filter(CIC_struct, input) 																									 \
{																																		 \
register float div_osr = 1.0f / (float)CIC_struct.OSR;																					 \
register float limit = (int32)(4294967296.0 / 2.0 * div_osr * div_osr);																	 \
CIC_struct.integrator[1] += CIC_struct.integrator[0];																				     \
CIC_struct.integrator[0] += (int32)fmaxf(fminf(input, limit), -limit);																     \
register float counter_temp = CIC_struct.counter + 1.0f;																			 	 \
counter_temp = (counter_temp - (float)((int32)(counter_temp * div_osr)) * (float)CIC_struct.OSR);									 	 \
CIC_struct.counter = (int32)counter_temp;																							 	 \
register int32 div_memory_new = (int32)(counter_temp * 20.0f * div_osr);															 	 \
if (CIC_struct.div_memory != div_memory_new)																						 	 \
{																																	 	 \
CIC_struct.div_memory = div_memory_new;																								 	 \
register int32* subtractor = (int32*)((int32(*)[2]) & CIC_struct.subtractor + div_memory_new);										 	 \
register int32* decimator = (int32*)&CIC_struct.decimator_memory + div_memory_new;													 	 \
* (subtractor + 1) = *decimator - *subtractor;																						 	 \
* subtractor = *decimator;																											 	 \
* decimator = CIC_struct.integrator[1];																								 	 \
CIC_struct.out = (float)(CIC_struct.integrator[1] - *subtractor - *(subtractor + 1)) * div_osr * div_osr;							 	 \
}																																	 	 \
}	


#define abc_abg(t_struct)								    \
{														    \
register float in_bc = t_struct.b + t_struct.c;				\
t_struct.alfa = (2.0f*t_struct.a - in_bc) * MATH_1_3;		\
t_struct.beta = (t_struct.b - t_struct.c) * MATH_1_SQRT3;   \
t_struct.gamma = MATH_1_3 * (t_struct.a + in_bc);			\
}

#define abg_abcn(t_struct)							 			 \
{													 			 \
t_struct.a = t_struct.alfa + t_struct.gamma;	           		 \
register float out_temp = t_struct.gamma - 0.5f * t_struct.alfa; \
register float t_struct_bet_temp = t_struct.beta * MATH_SQRT3_2; \
t_struct.b = out_temp + t_struct_bet_temp;						 \
t_struct.c = out_temp - t_struct_bet_temp;                       \
t_struct.neutro = t_struct.gamma * 3.0f;			                     \
}

#define abg_dqz(t_struct, angle)								 \
{                                            					 \
register float sine = sinf(angle);            					 \
register float cosine = cosf(angle);          					 \
t_struct.d =  cosine * (t_struct.alfa) + sine * (t_struct.beta); \
t_struct.q = -sine * (t_struct.alfa) + cosine * (t_struct.beta); \
t_struct.z = t_struct.gamma;									 \
}

#define dqz_abg(t_struct, angle)					 \
{                                   				 \
register float sine = sinf(angle);   				 \
register float cosine = cosf(angle);				 \
t_struct.alfa = cosine*t_struct.d - sine*t_struct.q; \
t_struct.beta = sine*t_struct.d + cosine*t_struct.q; \
t_struct.gamma = t_struct.z;						 \
}

#define reactive_power_abc(p_struct, u_t_struct, i_t_struct)			    \
{																		    \
p_struct.q_a = i_t_struct.a * (u_t_struct.b - u_t_struct.c) * MATH_1_SQRT3; \
p_struct.q_b = i_t_struct.b * (u_t_struct.c - u_t_struct.a) * MATH_1_SQRT3; \
p_struct.q_c = i_t_struct.c * (u_t_struct.a - u_t_struct.b) * MATH_1_SQRT3; \
p_struct.q_abc = p_struct.q_a + p_struct.q_b + p_struct.q_c;			    \
}

#define active_power_abc(p_struct, u_t_struct, i_t_struct)	 \
{															 \
p_struct.p_a = i_t_struct.a * u_t_struct.a;					 \
p_struct.p_b = i_t_struct.b * u_t_struct.b;					 \
p_struct.p_c = i_t_struct.c * u_t_struct.c;					 \
p_struct.p_abc = p_struct.p_a + p_struct.p_b + p_struct.p_c; \
}

#define power_abc(p_struct, u_t_struct, i_t_struct)	 \
active_power_abc(p_struct, u_t_struct, i_t_struct)	 \
reactive_power_abc(p_struct, u_t_struct, i_t_struct)



struct CIC_struct
{
	int32 integrator[2];
	int32 subtractor[20][2];
	int32 decimator_memory[20];
	float out;
	Uint32 div_memory;
	Uint32 counter;
	Uint32 OSR;
};

struct Power_struct
{
	float p_a;
	float p_b;
	float p_c;
	float p_abc;
	float q_a;
	float q_b;
	float q_c;
	float q_abc;
};

struct transformation_struct
{
	float a;
	float b;
	float c;
	float neutro;
	float alfa;
	float beta;
	float gamma;
	float d;
	float q;
	float z;
	float p;
	float n;
};

struct SOGI_struct
{
	float Ts;
	float x;
	float qx;
	float input_err;
};

struct PI_struct {
	float Kp;
	float Ts_Ti;
	float Ti;
	float Ts;
	float integrator_last;
	float integrator;
	float proportional;
	float lim_H;
	float lim_L;
	float out;
};

struct PR_struct
{
	float y0;
	float x0;
	float x1;
	float x2;
	float Kp;
	float Ki;
	float Ts;
	float out;
	float w;
	float lim_H;
	float lim_L;
	float alpha;
};

void SOGI_calc(struct SOGI_struct* SOGI, float input, float w);

void PR_calc(struct PR_struct* PR, float error);
void PR_calc_imp(struct PR_struct* PR, float error);

void PI_antiwindup_fast(struct PI_struct* PI, float error);
void PI_antiwindup(struct PI_struct* PI, float error);

#endif /* Controllers_H_ */
