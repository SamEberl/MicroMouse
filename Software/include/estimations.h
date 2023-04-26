#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include <vector>
#include "defs.h"
#include "classes.h"

#include <SDL2/SDL.h>

using namespace std;

class CellEst {
public:
    CellEst();
    void initialize(float row, float col);
    int get_row() const;
    int get_col() const; 
    vector<float> get_point(char pointNumber) const;
    bool has_wall(char direction) const;
    void update_wall(char direction, bool wallPresent);
    bool is_seen() const;
    void set_seen(bool value);

private:
    float row;
    float col;
    float N;
    float E;
    float S;
    float W;
    bool seen;
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class SensorEst {
    public:
        char looking_at; // is the sensor looking at a horizontal or vertical wall
        float dist_measure;
        vector<float> intersection;
        float offset_direction;
        float sens_dir;
        float offset_position;
        vector<float> sens_pos;

        SensorEst();
        void init(vector<float> robot_pos, float robot_dir, float offset_direction_, float offset_position_);
        void updatePosition(vector<float> robot_pos, float robot_dir);
        void getDistanceToWall(Sensor sensor);
        void compareDistanceToWall(SDL_Renderer *renderer, vector<float>& rob_pos, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
};

class RobotEst {
    public:
        vector<float> rob_pos;
        float rob_dir;
        float distance_wheels;
        float width;
        float height;
        SensorEst sensR;
        SensorEst sensR2;
        SensorEst sensS;
        SensorEst sensL2;
        SensorEst sensL;

        RobotEst(float x, float y, float direction_, float distance_wheels, float width, float height);
        void updatePosition(float SpeedL, float SpeedR);
        void compareDistances(SDL_Renderer *renderer, Robot mouse, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
        void get_offset_left(float& offset, float& theta_rel, float alpha, float beta);
        void get_offset_right(float& offset, float& theta_rel, float alpha, float beta);
        void get_offset_front_left(float& offset, float& theta_rel, float alpha);
        void get_offset_front_right(float& offset, float& theta_rel, float alpha);
        void horizontal_dist_to_west_wall(float error_band, float smoothing);

};

#endif
