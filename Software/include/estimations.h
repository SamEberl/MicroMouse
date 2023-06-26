#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "defs.h"

typedef struct {
    Point p1;
    Point p2;
    Point p3;
    Point p4;
} CornerEst;

void CornerEst_initialize(CornerEst* cornerEst, float row, float col);

typedef struct {
    float N;
    float E;
    float S;
    float W;
    bool is_seen;
    Point p1;
    Point p2;
    Point p3;
    Point p4;
} CellEst;

void CellEst_initialize(CellEst* cellEst, float row, float col);
void CellEst_update_wall(CellEst* cellEst, char direction, bool wallPresent);
bool CellEst_has_wall(const CellEst* cellEst, char direction);


typedef struct {
    char looking_at;
    float dist_measure;
    Point intersection;
    float offset_direction;
    float sens_dir;
    float offset_position;
    Point sens_pos;
} SensorEst;

void SensorEst_init(SensorEst* sensorEst, Point* robot_pos, float robot_dir, float offset_direction_, float offset_position_);
void SensorEst_updatePosition(SensorEst& sensorEst, Point& robot_pos, float robot_dir);
void compareDistanceToWall(SensorEst* sensor, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);

typedef struct RobotEst {
    Point rob_pos;
    float rob_dir;
    int8_t distance_wheels;
    int8_t width;
    int8_t height;
    int8_t speedLeft;
    int8_t speedRight;

    SensorEst sensR;
    SensorEst sensR2;
    SensorEst sensS;
    SensorEst sensL2;
    SensorEst sensL;
} RobotEst;

void RobotEst_init(RobotEst* rob, float x, float y, float direction_, float distance_wheels_, float width_, float height_);
void RobotEst_get_offset_and_angle(float* height, float* angle_rel, float dist_1, float dist_2, float alpha);
void localization(RobotEst* rob);
void RobotEst_updatePosition(RobotEst* rob, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {


#endif
