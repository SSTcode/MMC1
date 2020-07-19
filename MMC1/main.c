// MMC1.cpp : Define las funciones exportadas del archivo DLL.
//

#include "stdafx.h"


//! Debug :
//. Change the view to expert view using Tools > Settings > Expert settings.
//2. Attach the DLL to the PLECS process using Debug > Attach to process > PLECS.exe.
//3. In main.c, create a breakpoint at the line y = kp * e + ki * i.

#define DLL_INPUTS_NUMBER   1
#define DLL_OUTPUTS_NUMBER  1
#define DLL_PARAMETERS_NUMBER  1

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


}

//This function is automatically called every sample time
//output is written to DLL output port after the output delay
DLLEXPORT void plecsOutput(struct SimulationState* aState)
{
   
}
