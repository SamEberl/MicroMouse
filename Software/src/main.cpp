#include "defs.h"
#include "utils.h"
#include "estimations.h"
#include "algorithm.h"


#include "asw.h"
#include "bsw_api.h"
#include "general.h"
#include "xc.h"

static uint8_t statusAsw = 20;

void incrementStatus(void) {
    statusAsw++;
}

void asw_init(void){
    //TODO INITIALIZE ASW PROGRAM

    // registerButtonPressedCB(&incrementStatus);
    // registerButtonReleasedCB(&incrementStatus);

    int start_x = LABYRINTH_WIDTH-1;
    int start_y = LABYRINTH_HEIGHT-1;

    float rob_x = (start_x*(CELL_SIZE + WALL_WIDTH))+(CELL_SIZE/2)+WALL_WIDTH;
    float rob_y = (start_y*(CELL_SIZE + WALL_WIDTH))+(CELL_SIZE/2)+WALL_WIDTH;
    float rob_dir = M_PI*3/2;

    RobotEst mouseEst = RobotEst(rob_x, rob_y, rob_dir, DISTANCE_WHEELS, MOUSE_WIDTH, MOUSE_HEIGHT);
    CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT];
    CornerEst cornersEst[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1];
    init_labyrinth(labyrinthEst);
    init_corners(cornersEst);
    Planner planner(start_x, start_y);
}

void asw_10ms(void){
    //TODO 10MS PROCESS FOR ASW
    mouse.measureDistances(labyrinth, corners);

    Point wheel_speed;

    // Update the wheel_speed using the planner function
    planner_update(mouseEst, labyrinthEst, wheel_speed);

    // Set the motor velocities using the wheel_speed array elements
    setMotorVelocity(MOTOR_L, wheel_speed[0]);
    setMotorVelocity(MOTOR_R, wheel_speed[1]);


    mouseEst.updatePosition(renderer, mouse, labyrinthEst, cornersEst);
    mouseEst.localization();


    uint8_t sensorVal;
    getSensorVal(2,&sensorVal);
    if(sensorVal < 100){
        setLEDState(1,ON);
    }
    else{
        setLEDState(1,OFF);
    }
    getSensorVal(4,&sensorVal);
    if(sensorVal < 100){
        setLEDState(2,ON);
    }
    else{
        setLEDState(2,OFF);
    }
    setASWStatus(statusAsw);
}
