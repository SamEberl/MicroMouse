#ifndef VL6180X_H
#define VL6180X_H
#include "general.h"

typedef enum measurement_state{
    INIT,
    STATUS_RECEIVED,
    MEASUREMENT_STARTED,
    INTERRUPT_RECEIVED,
    RANGE_RECEIVED,
    INTERRUPT_CLEARED,
    DONE
}measurement_state;

typedef struct sensor{
    measurement_state state;
    uint8_t status;
    uint8_t intrpt;
    uint8_t distance;
    uint8_t initialized;
}sensor;

errorCode I2CTp_10ms(void);
errorCode init_vl6180x(uint8_t sensor_num);
errorCode getSensor1Data(uint8_t *data);
errorCode getSensor2Data(uint8_t *data);
errorCode getSensor3Data(uint8_t *data);
errorCode getSensor4Data(uint8_t *data);
errorCode getSensor5Data(uint8_t *data);
#endif

