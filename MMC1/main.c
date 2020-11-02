#include "stdafx.h"



//! Debug :
//. Change the view to expert view using Tools > Settings > Expert settings.
//2. Attach the DLL to the PLECS process using Debug > Attach to process > PLECS.exe.
//3. In main.c, create a breakpoint at the line y = kp * e + ki * i.

#define DLL_INPUTS_NUMBER   37
#define DLL_OUTPUTS_NUMBER  46
#define DLL_PARAMETERS_NUMBER  9

DLLEXPORT void plecsSetSizes(struct SimulationSizes* aSizes)
{
    aSizes->numInputs = DLL_INPUTS_NUMBER;
    aSizes->numOutputs = DLL_OUTPUTS_NUMBER;
    aSizes->numStates = 0;
    aSizes->numParameters = DLL_PARAMETERS_NUMBER;
}

//This function is automatically called at the beginning of the simulation
DLLEXPORT void plecsStart(struct SimulationState* aState)
{
    aState_global = aState;

    PLL = (const struct PLL_struct){ 0 };
    Ctrl = (const struct Control_struct){ 0 };

    PLL.Ts = Ctrl.Ts = aState->parameters[0];
    float L   = aState->parameters[1];
    float r   = aState->parameters[2];
    float Ldc = aState->parameters[3];
    float rdc = aState->parameters[4];
    float Lac = aState->parameters[5];
    float rac = aState->parameters[6];
    Ctrl.C   = aState->parameters[7];
    Ctrl.n_cell = aState->parameters[8];

    Ctrl.Vdc = 100.0f;
    Ctrl.Vx = Ctrl.Vdc * 0.5f;
    Ctrl.Vc_ref = Ctrl.Vx / Ctrl.n_cell;
    Ctrl.Exy_ref = Ctrl.Vc_ref * Ctrl.Vc_ref * Ctrl.C * 0.5f;
    float Io_max = 8.0f*50.0f;
    float Umax_i = Ctrl.Vc_ref * Ctrl.n_cell;
    float Umax_v = Ctrl.Vx * Io_max;
    float Umin_i = -Umax_i;
    float Umin_v = -Umax_v;

    PLL.omega_nominal = MATH_2PI * 50.0f;
    PLL.PI.Kp = 92.0f;
    PLL.PI.Ti = 0.087f;
    PLL.PI.lim_H = 400.0f;
    PLL.PI.lim_L = -400.0f;
    PLL.PI.Ts = PLL.Ts;
    PLL.SOGI_alf.Ts = PLL.Ts;
    PLL.SOGI_bet.Ts = PLL.Ts;

    /////////////////////////CIC FILTER///////////////////////////////////////
    int i;
    for (i = 0; i < 6; i++) {
        Ctrl.CIC_Ixy[i].OSR = 0.02f / Ctrl.Ts / 2.0f;//312;
        //Ctrl.CIC_load_q_b.OSR = Comp.CIC_load_q_a.OSR;
        //Ctrl.CIC_load_q_c.OSR = Comp.CIC_load_q_a.OSR;
    }


    /// ///////////////////ENEGY//////////////////////////////////////////////
    
    
    Ctrl.CIC_Ux_dc_p.OSR = 0.02f / Ctrl.Ts / 2.0f;//312;
    Ctrl.CIC_Ux_dc_n.OSR = Ctrl.CIC_Ux_dc_p.OSR;




    register float Uc_xy_filter_T = 2.0f * sqrtf(MATH_SQRT2 - 1.0f) / (MATH_2PI * 30.0f); //30Hz cutoff, Time constant of low-pass
    Ctrl.Uc_xy_filter_coeff = Ctrl.Ts * 2.0f / Uc_xy_filter_T;

    register float alfa2 = 2.0f;
    register float STC1 = Ctrl.C*0.01f*0.5f;// +Uc_xy_filter_T;
    register float ko = 31.5f* STC1;
    register float STC2 = 1.5f * Ctrl.Ts;// +Uc_xy_filter_T;
    float kp_dc = STC1 / (alfa2 * STC2 * ko);
    float ti_dc = alfa2 * alfa2 * STC2;

    Ctrl.PI_ov.Kp = kp_dc;
    Ctrl.PI_ov.Ki = kp_dc / ti_dc;
    Ctrl.PI_ov.Ts = Ctrl.Ts;
    Ctrl.PI_ov.lim_H = Umax_v;
    Ctrl.PI_ov.lim_L = Umin_v;

    Ctrl.PI_zv.Ti = kp_dc;
    Ctrl.PI_zv.Kp = kp_dc / ti_dc;
    Ctrl.PI_zv.Ts = Ctrl.Ts;
    Ctrl.PI_zv.lim_H = Umax_v;
    Ctrl.PI_zv.lim_L = Umin_v;

    Ctrl.PI_sv.Ti = kp_dc;
    Ctrl.PI_sv.Kp = kp_dc / ti_dc;
    Ctrl.PI_sv.Ts = Ctrl.Ts;
    Ctrl.PI_sv.lim_H = Umax_v;
    Ctrl.PI_sv.lim_L = Umin_v;

    Ctrl.PI_mv.Ti = kp_dc;
    Ctrl.PI_mv.Kp = kp_dc / ti_dc;
    Ctrl.PI_mv.Ts = Ctrl.Ts;
    Ctrl.PI_mv.lim_H = Umax_v;
    Ctrl.PI_mv.lim_L = Umin_v;
   ///////////////////////////////Current///////////////////////////////////////////////////////////////////////// 
   
    register float I_xy_filter_T = 2.0f * sqrtf(MATH_SQRT2 - 1.0f) / (MATH_2PI * 30.0f); //30Hz cutoff, Time constant of low-pass
    Ctrl.I_xy_filter_coeff = Ctrl.Ts * 2.0f / I_xy_filter_T;
    
    Ctrl.Is_est = 0.3f;
    
    
    float Lm = 3.0f*Ldc + L + 2.0f *Lac;
    float rm = 3.0f*rdc + r + 2.0f *rac;
    register float alfa_Im = 6.0f;

    register float STC_Im = (1.5f * Ctrl.Ts)/rm;
    float kp_Im = (Lm/rm) / (alfa_Im * STC_Im);
    float ti_Im = alfa_Im * alfa_Im * STC_Im;

    Ctrl.PI_Im.Kp = kp_Im;
    Ctrl.PI_Im.Ts_Ti = Ctrl.Ts / ti_Im;
    
    //float Ti = 6.0f;
    //float xi = 0.707f;
    //float h = Ctrl.Ts * Ti;
    ///// /////////////////////////////////////////////////////////////////////////////
    //float fim = 130.0f;
    //float wn_im = 2.0 * MATH_1_PI * fim;
    //float a1m = -2.0 * exp(-xi * wn_im * h) * cos(sqrt(1.0 - xi * xi) * wn_im * h);
    //float a2m = exp(-2.0 * xi * wn_im * h);
    //float ki_m = rm / h * (1.0 + a1m + a2m) / (1.0 - exp(-h * rm / Meas.Lm));
    //float kp_m = rm * (a1m - a2m + 1.0 + 2.0 * exp(-h * rm / Meas.Lm)) / (2.0 * (1.0 - exp(-h * rm / Meas.Lm)));
    //
    //Ctrl.PI_mi.lim_H =  Io_max;
    //Ctrl.PI_mi.lim_L = -Io_max;
    //
    //Ctrl.PI_mi.Kp = kp_m;
    //Ctrl.PI_mi.Ki = ki_m;
    //
    ///// ////////////////////////////////////////////////////////////////
    float Ls = 3.0f * Ldc + L ;
    float rs = 3.0f * rdc + r ;
    register float alfa_Is = 4.0f;

    register float STC_Is = (1.5f * Ctrl.Ts) / rs;
    float kp_Is = (Ls / rs) / (alfa_Is * STC_Is) / (Ctrl.n_cell * Ctrl.Vx);
    float ti_Is = alfa_Is * alfa_Is * STC_Is;

    Ctrl.PI_Is.Kp = kp_Is;
    Ctrl.PI_Is.Ts_Ti = Ctrl.Ts / ti_Is;
    Ctrl.PI_Is.lim_H = Io_max;
    Ctrl.PI_Is.lim_L = -Io_max;
    
   //float Ti = 4.0f;
   //float xi = 0.707f;
   //float h = Ctrl.Ts * Ti;
    ///// //////////////////////
    //float fis = 130.0f;
    //float wn_is = MATH_2PI * fis;
    //float a1s = -2.0 * exp(-xi * wn_is * h) * cos(sqrt(1.0 - xi * xi) * wn_is * h);
    //float a2s = exp(-2.0 * xi * wn_is * h);
    //float ki_s = rs / h * (1.0 + a1s + a2s) / (1.0 - exp(-h * rs / Meas.Ls));
    //float kp_s = rs * (a1s - a2s + 1.0 + 2.0 * exp(-h * rs / Meas.Ls)) / (2.0 * (1.0 - exp(-h * rs / Meas.Ls)));
    //
    //
    //Ctrl.PI_si.Kp = -kp_s;
    //Ctrl.PI_si.Ki = -ki_s;
    //Ctrl.PI_si.lim_H = Io_max*500.0f;
    //Ctrl.PI_si.lim_L = -Io_max*500.0f;

    /// ////////////////////////////////////////////////////////////////
    Meas.Lz = L;
    float rz = r;
    register float alfa_Iz = 2.0f;
    //PR
    Ctrl.PR_Iz.w = MATH_2PI * 100.0f;
    Ctrl.PR_Iz.Kp = Meas.Lz * Ctrl.PR_Iz.w * (alfa_Iz * alfa_Iz / sqrtf(alfa_Iz))-rz;
    Ctrl.PR_Iz.Ki = Meas.Lz * Ctrl.PR_Iz.w * Ctrl.PR_Iz.w * (alfa_Iz * alfa_Iz - 1.0f);
    //PI
    register float STC_Iz = (1.5f * Ctrl.Ts) / rz;
    float kp_Iz = (Meas.Lz / rz) / (alfa_Iz * STC_Iz);
    float ti_Iz = alfa_Iz * alfa_Iz * STC_Iz;
    
    Ctrl.PI_Izd.Kp = kp_Iz;
    Ctrl.PI_Izd.Ts_Ti = Ctrl.Ts / ti_Iz;
    Ctrl.PI_Izd.lim_H = Io_max;
    Ctrl.PI_Izd.lim_L = -Io_max;

    Ctrl.PI_Izq.Kp = kp_Iz;
    Ctrl.PI_Izq.Ts_Ti = Ctrl.Ts / ti_Iz;
    Ctrl.PI_Izq.lim_H = Io_max;
    Ctrl.PI_Izq.lim_L = -Io_max;
    //float Ti = 4.0f;
    //float xi = 0.707f;
    //float h = Ctrl.Ts * Ti;
    //float fiz = 200.0f;
    //float wn_iz = MATH_2PI * fiz;
    //float a1z = -2.0 * exp(-xi * wn_iz * h) * cos(sqrt(1.0 - xi * xi) * wn_iz * h);
    //float a2z = exp(-2.0 * xi * wn_iz * h);
    //float ki_z = rz / h * (1.0 + a1z + a2z) / (1.0 - exp(-h * rz / Meas.Lz));
    //float kp_z = rz * (a1z - a2z + 1.0 + 2.0 * exp(-h * rz / Meas.Lz)) / (2.0 * (1.0 - exp(-h * rz / Meas.Lz)));
    //
    //Ctrl.PI_zi_d.Kp = kp_z;
    //Ctrl.PI_zi_d.Ki = ki_z;
    //Ctrl.PI_zi_d.lim_H = Io_max * 100.0f;
    //Ctrl.PI_zi_d.lim_L = -Io_max * 100.0f;
    //
    //Ctrl.PI_zi_q.Kp = kp_z;
    //Ctrl.PI_zi_q.Ki = ki_z;
    //Ctrl.PI_zi_q.lim_H = Io_max * 100.0f;
    //Ctrl.PI_zi_q.lim_L = -Io_max * 100.0f;

    ///////////////////////////////////////////////////////////////
    Meas.Lo = L + 2.0f * Lac;
    float ro = r + 2.0f * rac;
    register float alfa_Io = 2.0f;
    //PR
    Ctrl.PR_Io.w = MATH_2PI * 50.0f;
    Ctrl.PR_Io.Kp = Meas.Lo * Ctrl.PR_Io.w * (alfa_Io * alfa_Io / sqrtf(alfa_Io)) - ro;
    Ctrl.PR_Io.Ki = Meas.Lo * Ctrl.PR_Io.w * Ctrl.PR_Io.w * (alfa_Io * alfa_Io - 1.0f);
    //PI
    register float STC_Io = (1.5f * Ctrl.Ts) / ro;
    float kp_Io = (Meas.Lo / ro) / (alfa_Io * STC_Io);
    float ti_Io = alfa_Io * alfa_Io * STC_Io;
    
    Ctrl.PI_Iod.Kp = kp_Io;
    Ctrl.PI_Iod.Ts_Ti = Ctrl.Ts / ti_Io;
    Ctrl.PI_Iod.lim_H = Io_max ;
    Ctrl.PI_Iod.lim_L = -Io_max;

    Ctrl.PI_Ioq.Kp = kp_Io;
    Ctrl.PI_Ioq.Ts_Ti = Ctrl.Ts / ti_Io;
    Ctrl.PI_Ioq.lim_H = Io_max;
    Ctrl.PI_Ioq.lim_L = -Io_max;
    
    //float Ti = 4.0f;
    //float xi = 0.707f;
    //float h = Ctrl.Ts * Ti;
    //float fio = 200.0f;
    //float wn_io = MATH_2PI * fio;
    //float a1o = -2.0 * exp(-xi * wn_io * h) * cos(sqrt(1.0 - xi * xi) * wn_io * h);
    //float a2o = exp(-2.0 * xi * wn_io * h);
    //float ki_o = ro / h * (1.0 + a1o + a2o) / (1.0 - exp(-h * ro / Meas.Lo));
    //float kp_o = ro * (a1o - a2o + 1.0 + 2.0 * exp(-h * ro / Meas.Lo)) / (2.0 * (1.0 - exp(-h * ro / Meas.Lo)));
    //
    //Ctrl.PI_oi_d.Kp = kp_o;
    //Ctrl.PI_oi_d.Ki = ki_o;
    //Ctrl.PI_oi_d.lim_H = Io_max*100.0f;
    //Ctrl.PI_oi_d.lim_L = -Io_max*100.0f;
    //
    //Ctrl.PI_oi_q.Kp = kp_o;
    //Ctrl.PI_oi_q.Ki = ki_o;
    //Ctrl.PI_oi_q.lim_H = Io_max*100.0f;
    //Ctrl.PI_oi_q.lim_L = -Io_max*100.0f;
    //



}

//This function is automatically called every sample time
//output is written to DLL output port after the output delay
DLLEXPORT void plecsOutput(struct SimulationState* aState)
{
    //6
    Meas.Vgrid[1] = aState_global->inputs[1];
    Meas.Vgrid[0] = aState_global->inputs[0];
    Meas.Vgrid[2] = aState_global->inputs[2];
    //
    Meas.Uy_grid.a = aState_global->inputs[0];
    Meas.Uy_grid.b = aState_global->inputs[1];
    Meas.Uy_grid.c = aState_global->inputs[2];
    //
    Meas.Iy_grid.a = aState_global->inputs[3];
    Meas.Iy_grid.b = aState_global->inputs[4];
    Meas.Iy_grid.c = aState_global->inputs[5];
   // // 4
    //Meas.Ux_dc.p = aState_global->inputs[6];
    //Meas.Ux_dc.n = aState_global->inputs[7];
    //Meas.Ix_dc.p = aState_global->inputs[8];
    //Meas.Ix_dc.n = aState_global->inputs[9];
    //6
    //Meas.Uxy_p.a = aState_global->inputs[10];
    //Meas.Uxy_p.b = aState_global->inputs[11];
    //Meas.Uxy_p.c = aState_global->inputs[12];
    //Meas.Uxy_n.a = aState_global->inputs[13];
    //Meas.Uxy_n.b = aState_global->inputs[14];
    //Meas.Uxy_n.c = aState_global->inputs[15];
    //6
    Meas.Ixy[0] = aState_global->inputs[16]; //ipa
    Meas.Ixy[1] = aState_global->inputs[17]; //ipb
    Meas.Ixy[2] = aState_global->inputs[18]; //ipc
    Meas.Ixy[3] = aState_global->inputs[19]; //ina
    Meas.Ixy[4] = aState_global->inputs[20]; //inb
    Meas.Ixy[5] = aState_global->inputs[21]; //inc
    //8
    Meas.Im_ref   = aState_global->inputs[22];
    Meas.Is_ref   = aState_global->inputs[23];
    Meas.Iz_step = aState_global->inputs[24];
    //Meas.Iz_ref.b = aState_global->inputs[25];
    //Meas.Iz_ref.c = aState_global->inputs[26];
    //Meas.Io_ref.a = aState_global->inputs[27];
    //Meas.Io_ref.b = aState_global->inputs[28];
    //Meas.Io_ref.c = aState_global->inputs[29];
    ////6
    //Meas.Uc_pn[0] = aState_global->inputs[30]; //vcpa
    //Meas.Uc_pn[1] = aState_global->inputs[31]; //vcpb
    //Meas.Uc_pn[2] = aState_global->inputs[32]; //vcpc
    //Meas.Uc_pn[3] = aState_global->inputs[33]; //vcna
    //Meas.Uc_pn[4] = aState_global->inputs[34]; //vcnb
    //Meas.Uc_pn[5] = aState_global->inputs[35]; //vcnc
    ////1
    Meas.Is= aState_global->inputs[36]; //vdc

    float enable = 1;
    PLL_calc(enable);
    Control_calc(PLL.RDY);

    aState_global->outputs[0] = Ctrl.Iz_struct.a;
    aState_global->outputs[1] = Ctrl.Iz_struct.b;
    aState_global->outputs[2] = Ctrl.Iz_struct.c;
    aState_global->outputs[3] = Ctrl.Io_struct.a;
    aState_global->outputs[4] = Ctrl.Io_struct.b;
    aState_global->outputs[5] = Ctrl.Io_struct.c;
    aState_global->outputs[6] = Ctrl.Is;
    aState_global->outputs[7] = Ctrl.Im;
    //Sincronization 5+1+2
   aState_global->outputs[8]  = Ctrl.Io_ref.a;
   aState_global->outputs[9]  = Ctrl.Io_ref.b;
   aState_global->outputs[10] = Ctrl.Io_ref.c;
   aState_global->outputs[11] = PLL.theta_1;
   aState_global->outputs[12] = PLL.theta_2;
   aState_global->outputs[13] = PLL.RDY;// Ctrl.Vx;//Auxiliar
   aState_global->outputs[14] = Meas.Iy_grid.d;
   aState_global->outputs[15] = Meas.Iy_grid.q;
   //Sincronization 5+1+2
   aState_global->outputs[16] = Meas.Iz_ref.a;
   aState_global->outputs[17] = Meas.Iz_ref.b;
   aState_global->outputs[18] = Meas.Iz_ref.c;
   aState_global->outputs[19] = PLL.theta_4 ;
   aState_global->outputs[20] = PLL.theta_5 ;
   aState_global->outputs[21] = Ctrl.Vc_ref;//Auxiliar
   aState_global->outputs[22] = Ctrl.Iz_struct.d;
   aState_global->outputs[23] = Ctrl.Iz_struct.q;
   // OUT 5
   aState_global->outputs[24] = Ctrl.PI_Iod.out;
   aState_global->outputs[25] = Ctrl.PI_Ioq.out;
   aState_global->outputs[26] = Ctrl.PI_Izd.out;
   aState_global->outputs[27] = Ctrl.PI_Izq.out;
   aState_global->outputs[28] = Ctrl.PI_Is.out;
   // DUTY 6
   aState_global->outputs[29] = Ctrl.duty_modxy[0];
   aState_global->outputs[30] = Ctrl.duty_modxy[1];
   aState_global->outputs[31] = Ctrl.duty_modxy[2];
   aState_global->outputs[32] = Ctrl.duty_modxy[3];
   aState_global->outputs[33] = Ctrl.duty_modxy[4];
   aState_global->outputs[34] = Ctrl.duty_modxy[5];
   // ERROR 5
   aState_global->outputs[35] = Ctrl.Io_ref.d;
   aState_global->outputs[36] = Ctrl.Io_ref.q;
   aState_global->outputs[37] = Ctrl.Iz_ref.d;
   aState_global->outputs[38] = Ctrl.Iz_ref.q;
   aState_global->outputs[39] = Meas.Is_ref;
   //6
   aState_global->outputs[40] = Ctrl.xy2Dec.pa;
   aState_global->outputs[41] = Ctrl.xy2Dec.pb;
   aState_global->outputs[42] = Ctrl.xy2Dec.pc;
   aState_global->outputs[43] = Ctrl.xy2Dec.na;
   aState_global->outputs[44] = Ctrl.xy2Dec.nb;
   aState_global->outputs[45] = Ctrl.xy2Dec.nc;




}
