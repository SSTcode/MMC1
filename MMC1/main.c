#include "stdafx.h"



//! Debug :
//. Change the view to expert view using Tools > Settings > Expert settings.
//2. Attach the DLL to the PLECS process using Debug > Attach to process > PLECS.exe.
//3. In main.c, create a breakpoint at the line y = kp * e + ki * i.

#define DLL_INPUTS_NUMBER   30
#define DLL_OUTPUTS_NUMBER  8
#define DLL_PARAMETERS_NUMBER  7

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

    
    PLL.omega_nominal = MATH_2PI * 50.0f;
    PLL.PI.Kp = 92.0f;
    PLL.PI.Ti = 0.087f;
    PLL.PI.lim_H = 400.0f;
    PLL.PI.lim_L = -400.0f;
    PLL.PI.Ts = PLL.Ts;
    PLL.SOGI_alf.Ts = PLL.Ts;
    PLL.SOGI_bet.Ts = PLL.Ts;

    ////////////////////////////////////////////////////////////////

    Ctrl.CIC_Ux_dc_p.OSR = 0.02f / Ctrl.Ts / 2.0f;//312;
    Ctrl.CIC_Ux_dc_n.OSR = Ctrl.CIC_Ux_dc_p.OSR;

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
    Meas.Iy_grid.a = aState_global->inputs[3];
    Meas.Iy_grid.b = aState_global->inputs[4];
    Meas.Iy_grid.c = aState_global->inputs[5];
    // 4
    Meas.Ux_dc.p = aState_global->inputs[6];
    Meas.Ux_dc.n = aState_global->inputs[7];
    Meas.Ix_dc.p = aState_global->inputs[8];
    Meas.Ix_dc.n = aState_global->inputs[9];
    //6
    Meas.Uxy_p.a = aState_global->inputs[10];
    Meas.Uxy_n.a = aState_global->inputs[11];
    Meas.Uxy_p.b = aState_global->inputs[12];
    Meas.Uxy_n.b = aState_global->inputs[13];
    Meas.Uxy_p.c = aState_global->inputs[14];
    Meas.Uxy_n.c = aState_global->inputs[15];
    //6
    Meas.Ixy_p.a = aState_global->inputs[16];
    Meas.Ixy_n.a = aState_global->inputs[17];
    Meas.Ixy_p.b = aState_global->inputs[18];
    Meas.Ixy_n.b = aState_global->inputs[19];
    Meas.Ixy_p.c = aState_global->inputs[20];
    Meas.Ixy_n.c = aState_global->inputs[21];
    //8
    Meas.Im_ref   = aState_global->inputs[22];
    Meas.Is_ref   = aState_global->inputs[23];
    Meas.Iz_ref.a = aState_global->inputs[24];
    Meas.Iz_ref.b = aState_global->inputs[25];
    Meas.Iz_ref.c = aState_global->inputs[26];
    Meas.Io_ref.a = aState_global->inputs[27];
    Meas.Io_ref.b = aState_global->inputs[28];
    Meas.Io_ref.c = aState_global->inputs[29];


    float enable = 1;
    PLL_calc(enable);
    Control_calc(PLL.RDY);

    aState_global->outputs[0] = Ctrl.Ms;;
    aState_global->outputs[1] = Ctrl.Mm;
    aState_global->outputs[2] = Ctrl.Mo.a ;
    aState_global->outputs[3] = Ctrl.Mo.b ;
    aState_global->outputs[4] = Ctrl.Mo.c ;
    aState_global->outputs[5] = Ctrl.Mz.a ;
    aState_global->outputs[6] = Ctrl.Mz.b ;
    aState_global->outputs[7] = Ctrl.Mz.c ;


}
