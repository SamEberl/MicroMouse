#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "defs.h"

typedef struct {
    float x;
    float y;
} Point;

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
void SensorEst_updatePosition(SensorEst* sensorEst, Point* robot_pos, float robot_dir);
void compareDistanceToWall(SensorEst& sensor, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);

typedef struct RobotEst {
    Point rob_pos;
    float rob_dir;
    float distance_wheels;
    float width;
    float height;

    SensorEst sensR;
    SensorEst sensR2;
    SensorEst sensS;
    SensorEst sensL2;
    SensorEst sensL;
} RobotEst;

void RobotEst_init(RobotEst* rob, float x, float y, float direction_, float distance_wheels_, float width_, float height_);
void RobotEst_get_offset_and_angle(float* height, float* angle_rel, float dist_1, float dist_2, float alpha);
void localization(RobotEst* rob);

//-------------------------------------------------------------------------------------

class CornerEst {
public:
    CornerEst();
    void initialize(float row, float col);
    vector<float> get_point(char pointNumber) const;

private:
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class CellEst {
public:
    CellEst();
    void initialize(float row, float col);
    void update_wall(char direction, bool wallPresent);
    bool has_wall(char direction) const;
    vector<float> get_point(char pointNumber) const;

    bool is_seen;
    float threshold_seen;
    float N;
    float E;
    float S;
    float W;

private:
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class SensorEst {
    public:
        SensorEst();
        void init(vector<float> robot_pos, float robot_dir, float offset_direction_, float offset_position_);
        void updatePosition(vector<float> robot_pos, float robot_dir);
        void getDistanceToWall(Sensor sensor);
        void compareDistanceToWall(SDL_Renderer *renderer, vector<float>& rob_pos, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);

        char looking_at; // which wall is the sensor looking at. North (N), East (E), South (S), West(W)
        float dist_measure;
        float offset_direction;
        float sens_dir;
        float offset_position;
        vector<float> sens_pos;
        vector<float> intersection;


};

class RobotEst {
    public:
        RobotEst(float x, float y, float direction_, float distance_wheels, float width, float height);
        void updatePosition(SDL_Renderer *renderer, Robot mouse, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);
        void get_wheelspeeds(Robot mouse);
        void get_offset_and_angle(float& height, float& angle_rel, float dist_1, float dist_2, float alpha);
        void localization();

        float rob_dir;
        float distance_wheels;
        float width;
        float height;
        float speedLeft;
        float speedRight;
        SensorEst sensR;
        SensorEst sensR2;
        SensorEst sensS;
        SensorEst sensL2;
        SensorEst sensL;
        vector<float> rob_pos;
};

#endif
