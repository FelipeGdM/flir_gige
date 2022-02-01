#ifndef __THERMAL_PROC_DYN_GONFIG_H__
#define __THERMAL_PROC_DYN_GONFIG_H__

#include <cstdint>

typedef struct {
  int celsius_min;
  int celsius_max;
} ThermalProcDynConfig;

#endif  // __THERMAL_PROC_DYN_GONFIG_H__