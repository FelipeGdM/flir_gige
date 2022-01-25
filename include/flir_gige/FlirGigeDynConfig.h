#ifndef __FLIR_GIGE_DYN_GONFIG_H__
#define __FLIR_GIGE_DYN_GONFIG_H__

#include <cstdint>

typedef struct{
  bool raw;
  bool nuc_action;
  int nuc_mode;
  int fps;
} FlirGigeDynConfig;

#endif // __FLIR_GIGE_DYN_GONFIG_H__
