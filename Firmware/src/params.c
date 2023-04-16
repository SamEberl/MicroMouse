#include "params.h"

static uint16_t parameterValues[NUM_PARAMETERS] = PARAM_DEFAULTS;

errorCode getParameter(parameters param, uint16_t *data){
    uint8_t idx = (uint8_t) param;
    if((idx<0) || (idx>=NUM_PARAMETERS)) return OUT_OF_BOUNDS_ERROR;
    data[0] = parameterValues[idx];
    return OK;
}

errorCode setParameter(parameters param, uint16_t data){
    uint8_t idx = (uint8_t) param;
    if((idx<0) || (idx>=NUM_PARAMETERS)) return OUT_OF_BOUNDS_ERROR;
    parameterValues[idx] = data;
    return OK;
}