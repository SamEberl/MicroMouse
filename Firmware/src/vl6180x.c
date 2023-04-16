/*
 * File:   vl6180x.c
 * Author: louis
 *
 * Created on January 21, 2023, 9:31 PM
 */


#include "xc.h"
#include "vl6180x.h"
#include "general.h"
#include "i2cComm.h"
#include "ioconfig.h"
#include <stdlib.h>

static uint8_t measurement_running = FALSE;
static sensor sensors[5] = {0};
static uint8_t current_sensor = 0;
static uint8_t vl6180x_data = 0x00;

const uint8_t SLAVE_ADDR[5] = {0x30u,0x31u,0x32u,0x33u,0x34u};

errorCode updateMeasurement(void);

errorCode enableSensor(uint8_t active_sensor_num) {
    int i;
    
    switch(active_sensor_num){
        case 1:
            SEN1 = SENON;
            break;
        case 2:
            SEN2 = SENON;
            break;
        case 3:
            SEN3 = SENON;
            break;
        case 4:
            SEN4 = SENON;
            break;
        case 5:
            SEN5 = SENON;
            break;
        default:
            break;
    }
    for (i = 0; i < 30000; i++); // short dirty delay for changes to take effect,
    return OK;
}

errorCode init_vl6180x(uint8_t sensor_num)
{   
    enableSensor(sensor_num);
    
    //Mandatory Private Settings
    vl6180x_data = 0x01;
    I2C_transmit_blocking(0x29u,0x0207u,&vl6180x_data,1);
    vl6180x_data = 0x01;
    I2C_transmit_blocking(0x29u,0x0208u,&vl6180x_data,1);
    vl6180x_data = 0x00;
    I2C_transmit_blocking(0x29u,0x0096u,&vl6180x_data,1);
    vl6180x_data = 0xfd;
    I2C_transmit_blocking(0x29u,0x0097u,&vl6180x_data,1);
    vl6180x_data = 0x01;
    I2C_transmit_blocking(0x29u,0x00e3u,&vl6180x_data,1);
    vl6180x_data = 0x03;
    I2C_transmit_blocking(0x29u,0x00e4u,&vl6180x_data,1);
    vl6180x_data = 0x02;
    I2C_transmit_blocking(0x29u,0x00e5u,&vl6180x_data,1);
    vl6180x_data = 0x01;
    I2C_transmit_blocking(0x29u,0x00e6u,&vl6180x_data,1);
    vl6180x_data = 0x03;
    I2C_transmit_blocking(0x29u,0x00e7u,&vl6180x_data,1);
    vl6180x_data = 0x02;
    I2C_transmit_blocking(0x29u,0x00f5u,&vl6180x_data,1);
    vl6180x_data = 0x05;
    I2C_transmit_blocking(0x29u,0x00d9u,&vl6180x_data,1);
    vl6180x_data = 0xce;
    I2C_transmit_blocking(0x29u,0x00dbu,&vl6180x_data,1);
    vl6180x_data = 0x03;
    I2C_transmit_blocking(0x29u,0x00dcu,&vl6180x_data,1);
    vl6180x_data = 0xf8;
    I2C_transmit_blocking(0x29u,0x00ddu,&vl6180x_data,1);
    vl6180x_data = 0x00;
    I2C_transmit_blocking(0x29u,0x009fu,&vl6180x_data,1);
    vl6180x_data = 0x3c;
    I2C_transmit_blocking(0x29u,0x00a3u,&vl6180x_data,1);
    vl6180x_data = 0x00;
    I2C_transmit_blocking(0x29u,0x00b7u,&vl6180x_data,1);
    vl6180x_data = 0x03c;
    I2C_transmit_blocking(0x29u,0x00bbu,&vl6180x_data,1);
    vl6180x_data = 0x09;
    I2C_transmit_blocking(0x29u,0x00b2u,&vl6180x_data,1);
    vl6180x_data = 0x09;
    I2C_transmit_blocking(0x29u,0x00cau,&vl6180x_data,1);
    vl6180x_data = 0x01;
    I2C_transmit_blocking(0x29u,0x0198u,&vl6180x_data,1);
    vl6180x_data = 0x17;
    I2C_transmit_blocking(0x29u,0x01b0u,&vl6180x_data,1);
    vl6180x_data = 0x00;
    I2C_transmit_blocking(0x29u,0x01adu,&vl6180x_data,1);
    vl6180x_data = 0x05;
    I2C_transmit_blocking(0x29u,0x00ffu,&vl6180x_data,1);
    vl6180x_data = 0x05;
    I2C_transmit_blocking(0x29u,0x0100u,&vl6180x_data,1);
    vl6180x_data = 0x05;
    I2C_transmit_blocking(0x29u,0x0199u,&vl6180x_data,1);
    vl6180x_data = 0x1b;
    I2C_transmit_blocking(0x29u,0x01a6u,&vl6180x_data,1);
    vl6180x_data = 0x3e;
    I2C_transmit_blocking(0x29u,0x01acu,&vl6180x_data,1);
    vl6180x_data = 0x1f;
    I2C_transmit_blocking(0x29u,0x01a7u,&vl6180x_data,1);
    vl6180x_data = 0x00;
    I2C_transmit_blocking(0x29u,0x0030u,&vl6180x_data,1);

    //Recommended Public Settings
    vl6180x_data = 0x10;
    I2C_transmit_blocking(0x29u,0x0011u,&vl6180x_data,1);
    vl6180x_data = 0x30;
    I2C_transmit_blocking(0x29u,0x010au,&vl6180x_data,1);
    vl6180x_data = 0x46;
    I2C_transmit_blocking(0x29u,0x003fu,&vl6180x_data,1);
    vl6180x_data = 0xFF;
    I2C_transmit_blocking(0x29u,0x0031u,&vl6180x_data,1);
    vl6180x_data = 0x63;
    I2C_transmit_blocking(0x29u,0x0041u,&vl6180x_data,1);
    vl6180x_data = 0x01;
    I2C_transmit_blocking(0x29u,0x002eu,&vl6180x_data,1);
    vl6180x_data = 0x09;
    I2C_transmit_blocking(0x29u,0x001bu,&vl6180x_data,1);
    vl6180x_data = 0x31;
    I2C_transmit_blocking(0x29u,0x003eu,&vl6180x_data,1);
    vl6180x_data = 0x24;
    I2C_transmit_blocking(0x29u,0x0014u,&vl6180x_data,1);

    //Save Settings
    vl6180x_data = 0x00;
    I2C_transmit_blocking(0x29u,0x0016u,&vl6180x_data,1);
    
    //Change Address
    vl6180x_data = SLAVE_ADDR[sensor_num-1];
    I2C_transmit_blocking(0x29u,0x0212u,&vl6180x_data,1);
    
    sensors[sensor_num-1].initialized = 1u;
    return OK;
}

uint8_t getNextSensor() {
    uint8_t next_sensor = current_sensor;
    uint8_t found = 0;
    int i;
    //Check if all DONE
    for(i=0;i<5;i++) {
        if(sensors[i].state != DONE) break;
    }
    if(i==5) return 5;
    while(!found) {
        next_sensor++;
        if(next_sensor>=5) next_sensor = 0;
        if((sensors[next_sensor].initialized==1) && (sensors[next_sensor].state != DONE)) found = 1;
        else found = 0;
    }
    return next_sensor;
}

errorCode meas_TX_CB(errorCode status) {
    if(status == OK){
        return updateMeasurement();
    }
    //TODO: Error in Else Path
    return GENERAL_ERROR;
}

errorCode meas_RX_CB(errorCode status, uint8_t *data, uint8_t len) {
    if(len!=1){
        return INVALID_ARGUMENT_ERROR;
    }
    switch(sensors[current_sensor].state) {
        case STATUS_RECEIVED:
            sensors[current_sensor].status = data[0];
            break;
        case INTERRUPT_RECEIVED:
            sensors[current_sensor].intrpt = data[0];
            break;
        case RANGE_RECEIVED:
            sensors[current_sensor].distance = data[0];
            break;
        default:
            break;
    }
    return updateMeasurement();
}

errorCode updateMeasurement() {
    uint8_t next_sensor = getNextSensor();
    if(next_sensor==5){
        measurement_running = FALSE;
        return OK;
    }
    switch(sensors[next_sensor].state) {
        case INIT:
            I2C_receive(SLAVE_ADDR[next_sensor],0x04d,&vl6180x_data,1,&meas_RX_CB,1);
            sensors[next_sensor].state = STATUS_RECEIVED;
            break;
        case STATUS_RECEIVED:
            if(sensors[next_sensor].status&0x01) {
                vl6180x_data = 0x01;
                I2C_transmit(SLAVE_ADDR[next_sensor],0x018,&vl6180x_data,1,&meas_TX_CB,1);
                sensors[next_sensor].state = MEASUREMENT_STARTED;
            }
            else {
               I2C_receive(SLAVE_ADDR[next_sensor],0x04d,&vl6180x_data,1,&meas_RX_CB,1); 
            }
            break;
        case MEASUREMENT_STARTED:
            I2C_receive(SLAVE_ADDR[next_sensor],0x04f,&vl6180x_data,1,&meas_RX_CB,1);
            sensors[next_sensor].state = INTERRUPT_RECEIVED;
            break;
        case INTERRUPT_RECEIVED:
            if(sensors[next_sensor].intrpt&0x04) {
                I2C_receive(SLAVE_ADDR[next_sensor],0x062,&vl6180x_data,1,&meas_RX_CB,1);
                sensors[next_sensor].state = RANGE_RECEIVED;
            }
            else {
               I2C_receive(SLAVE_ADDR[next_sensor],0x04f,&vl6180x_data,1,&meas_RX_CB,1);
               sensors[next_sensor].state = INTERRUPT_RECEIVED;
            }
            break;
        case RANGE_RECEIVED:
            vl6180x_data = 0x07;
            I2C_transmit(SLAVE_ADDR[next_sensor],0x015,&vl6180x_data,1,&meas_TX_CB,1);
            sensors[next_sensor].state = DONE;
            break;
        default:
            return INVALID_ARGUMENT_ERROR;
            break;
    }
    current_sensor = next_sensor;
    return OK;
}

errorCode init_measurement(){
    int i;
    for(i=0;i<5;i++){
        if(sensors[i].initialized) sensors[i].state = INIT;
        else sensors[i].state = DONE;
        sensors[i].status = 0;
        sensors[i].intrpt = 0;
    }
    return OK;
}

errorCode I2CTp_10ms() {
    if(measurement_running == FALSE) {
        measurement_running = TRUE;
        init_measurement();
        current_sensor = 0;
        updateMeasurement();
    }
    return OK;
}

errorCode getSensor1Data(uint8_t *data) {
    data[0] = sensors[0].distance;
    return OK;
}

errorCode getSensor2Data(uint8_t *data) {
    data[0] = sensors[1].distance;
    return OK;
}

errorCode getSensor3Data(uint8_t *data) {
    data[0] = sensors[2].distance;
    return OK;
}

errorCode getSensor4Data(uint8_t *data) {
    data[0] = sensors[3].distance;
    return OK;
}

errorCode getSensor5Data(uint8_t *data) {
    data[0] = sensors[4].distance;
    return OK;
}