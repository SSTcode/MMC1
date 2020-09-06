#include "stdafx.h"



//! Debug :
//. Change the view to expert view using Tools > Settings > Expert settings.
//2. Attach the DLL to the PLECS process using Debug > Attach to process > PLECS.exe.
//3. In main.c, create a breakpoint at the line y = kp * e + ki * i.

#define DLL_INPUTS_NUMBER   37
#define DLL_OUTPUTS_NUMBER  22
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

    if (Ctrl.Vdc < 1.0) Ctrl.Vdc = 1.0f;
    Ctrl.Vx = Ctrl.Vdc * 0.5f;
    if (Ctrl.Vc_ref < Ctrl.Vdc / 4.0f * 1.2f) Ctrl.Vc_ref = Ctrl.Vdc / 4.0f * 1.2f;
    Ctrl.Exy_ref = Ctrl.Vc_ref * Ctrl.Vc_ref * Ctrl.C * 0.5f;
    float Io_max = 8.0f;
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

    ////////////////////////////////////////////////////////////////


Ctrl.U_dc=200;
Ctrl.Vdc = 100.0f;

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
    register float alfa_Im = 2.0f;
    register float STC_Im = (1.5f * Ctrl.Ts)/rm;
    float kp_Im = (Lm/rm) / (alfa_Im * STC_Im);
    float ti_Im = alfa_Im * alfa_Im * STC_Im;

    Ctrl.PI_Im.Kp = kp_Im;
    Ctrl.PI_Im.Ts_Ti = Ctrl.Ts / ti_Im;

    float Ls = 3.0f * Ldc + L ;
    float rs = 3.0f * rdc + r ;
    register float alfa_Is = 2.0f;
    register float STC_Is = (1.5f * Ctrl.Ts) / rs;
    float kp_Is = (Ls / rs) / (alfa_Is * STC_Is);
    float ti_Is = alfa_Is * alfa_Is * STC_Is;

    Ctrl.PI_Is.Kp = kp_Is;
    Ctrl.PI_Is.Ts_Ti = Ctrl.Ts / ti_Is;


    float Lz = L;
    float rz = r;
    register float alfa_Iz = 2.0f;
    Ctrl.PR_Iz.w = MATH_2PI * 100.0f;
    Ctrl.PR_Iz.Kp = Lz * Ctrl.PR_Iz.w * (alfa_Iz * alfa_Iz / sqrtf(alfa_Iz))-rz;
    Ctrl.PR_Iz.Ki = Lz* Ctrl.PR_Iz.w * Ctrl.PR_Iz.w * (alfa_Iz * alfa_Iz - 1.0f);

    float Lo = L + 2.0f * Lac;
    float ro = r + 2.0f * rac;
    register float alfa_Io = 2.0f;
    Ctrl.PR_Io.w = MATH_2PI * 50.0f;
    Ctrl.PR_Io.Kp = Lo * Ctrl.PR_Io.w * (alfa_Io * alfa_Io / sqrtf(alfa_Io)) - ro;
    Ctrl.PR_Io.Ki = Lo * Ctrl.PR_Io.w * Ctrl.PR_Io.w * (alfa_Io * alfa_Io - 1.0f);

}

//This function is automatically called every sample time
//output is written to DLL output port after the output delay
DLLEXPORT void plecsOutput(struct SimulationState* aState)
{
    //6
    Meas.Uy_grid.a = aState_global->inputs[0];
    Meas.Uy_grid.b = aState_global->inputs[1];
    Meas.Uy_grid.c = aState_global->inputs[2];

    Meas.Vgrid[0] = aState_global->inputs[0];
    Meas.Vgrid[1] = aState_global->inputs[1];
    Meas.Vgrid[2] = aState_global->inputs[2];

    Meas.Iy_grid.a = aState_global->inputs[3];
    Meas.Iy_grid.b = aState_global->inputs[4];
    Meas.Iy_grid.c = aState_global->inputs[5];
    // 4
    //Meas.Ux_dc.p = aState_global->inputs[6];
    //Meas.Ux_dc.n = aState_global->inputs[7];
    //Meas.Ix_dc.p = aState_global->inputs[8];
    //Meas.Ix_dc.n = aState_global->inputs[9];
    //6
    Meas.Uxy_p.a = aState_global->inputs[10];
    Meas.Uxy_p.b = aState_global->inputs[11];
    Meas.Uxy_p.c = aState_global->inputs[12];
    Meas.Uxy_n.a = aState_global->inputs[13];
    Meas.Uxy_n.b = aState_global->inputs[14];
    Meas.Uxy_n.c = aState_global->inputs[15];
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
    Meas.Iz_ref.a = aState_global->inputs[24];
    Meas.Iz_ref.b = aState_global->inputs[25];
    Meas.Iz_ref.c = aState_global->inputs[26];
    Meas.Io_ref.a = aState_global->inputs[27];
    Meas.Io_ref.b = aState_global->inputs[28];
    Meas.Io_ref.c = aState_global->inputs[29];
    //6
    Meas.Uc_pn[0] = aState_global->inputs[30]; //vcpa
    Meas.Uc_pn[1] = aState_global->inputs[31]; //vcpb
    Meas.Uc_pn[2] = aState_global->inputs[32]; //vcpc
    Meas.Uc_pn[3] = aState_global->inputs[33]; //vcna
    Meas.Uc_pn[4] = aState_global->inputs[34]; //vcnb
    Meas.Uc_pn[5] = aState_global->inputs[35]; //vcnc
    //1
    Meas.Vdc= aState_global->inputs[36]; //vdc

    float enable = 1;
    PLL_calc(enable);
    Control_calc(PLL.RDY);

   //aState_global->outputs[0] = Ctrl.xy2Dec.xy[0];
   //aState_global->outputs[1] = Ctrl.xy2Dec.xy[1];
   //aState_global->outputs[2] = Ctrl.xy2Dec.xy[2];
   //                            
   //aState_global->outputs[3] = Ctrl.xy2Dec.xy[3];
   //aState_global->outputs[4] = Ctrl.xy2Dec.xy[4];
   //aState_global->outputs[5] = Ctrl.xy2Dec.xy[5];
                              
    aState_global->outputs[6] = Ctrl.err_ov[0];
    aState_global->outputs[7] = Ctrl.err_ov[1];
    aState_global->outputs[8] = Ctrl.err_ov[2];

    aState_global->outputs[9]  = Ctrl.err_zv[0];
    aState_global->outputs[10] = Ctrl.err_zv[1];
    aState_global->outputs[11] = Ctrl.err_zv[2];

    aState_global->outputs[12] = Ctrl.err_sv;
    aState_global->outputs[13] = Ctrl.err_mv;
 
    aState_global->outputs[14] = Ctrl.PI_sv.out;
    aState_global->outputs[15] = Ctrl.PI_mv.out;

    aState_global->outputs[16] = Ctrl.PI_ov.out_3ph[0];
    aState_global->outputs[17] = Ctrl.PI_ov.out_3ph[1];
    aState_global->outputs[18] = Ctrl.PI_ov.out_3ph[2];

    aState_global->outputs[19] = Ctrl.PI_zv.out_3ph[0];
    aState_global->outputs[20] = Ctrl.PI_zv.out_3ph[1];
    aState_global->outputs[21] = Ctrl.PI_zv.out_3ph[2];


}
