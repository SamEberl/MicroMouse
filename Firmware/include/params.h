#ifndef PARAMS_H
#define PARAMS_H

#include "general.h"
#include <xc.h>

typedef enum parameters{
            PARAM_BSW_L_KP = 0,
            PARAM_BSW_L_KI = 1,
            PARAM_BSW_L_KD = 2,
            PARAM_BSW_R_KP = 3,
            PARAM_BSW_R_KI = 4,
            PARAM_BSW_R_KD = 5,
            PARAM_DUMMY = 6
}parameters;

#define NUM_PARAMETERS 7u

#define PARAM_DEFAULTS {0,0,0,0,0,0,0}

errorCode getParameter(parameters param, uint16_t *data);
errorCode setParameter(parameters param, uint16_t data);

#endif