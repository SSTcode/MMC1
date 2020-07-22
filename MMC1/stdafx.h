// pch.h: este es un archivo de encabezado precompilado.
// Los archivos que se muestran a continuación se compilan solo una vez, lo que mejora el rendimiento de la compilación en futuras compilaciones.
// Esto también afecta al rendimiento de IntelliSense, incluida la integridad del código y muchas funciones de exploración del código.
// Sin embargo, los archivos que se muestran aquí se vuelven TODOS a compilar si alguno de ellos se actualiza entre compilaciones.
// No agregue aquí los archivos que se vayan a actualizar con frecuencia, ya que esto invalida la ventaja de rendimiento.

// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"
#include "dllheader.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files
#include <windows.h>

typedef UINT16 Uint16;
typedef UINT32 Uint32;
typedef UINT64 Uint64;
typedef INT64 int64;
typedef INT32 int32;
typedef INT16 int16;

#include "Controllers.h"
#include "Control.h"
#include "PLL.h"

struct Measurements
{
	struct transformation_struct U_grid;
	struct transformation_struct I_grid;
	struct transformation_struct Ixy_p;
	struct transformation_struct Ixy_n;
	struct transformation_struct Io;
	struct transformation_struct Iz;
	float Im;
	float Is;
	float theta;
	float I_comp_N;
	float U_dc;
	struct Power_struct power_load_CIC_avg;
	struct Power_struct power_load;

};

extern struct Measurements Meas;
extern float duty[4];
extern struct SimulationState* aState_global;
// reference additional
