#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include <vector>
#include "defs.h"
#include "classes.h"

#include <SDL2/SDL.h>

using namespace std;

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
        void readMeasurements(Robot mouse);
        void get_offset_left(float& offset, float& theta_rel, float alpha);
        void get_offset_right(float& offset, float& theta_rel, float alpha);
        void get_offset_front_left(float& offset, float& theta_rel, float alpha);
        void get_offset_front_right(float& offset, float& theta_rel, float alpha);
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
