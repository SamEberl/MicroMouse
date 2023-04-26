#ifndef CLASSES_H
#define CLASSES_H

#include <vector>
#include "defs.h"

#include <SDL2/SDL.h>

using namespace std;

class Cell {
public:
    Cell();
    void initialize(float row, float col);
    int get_row() const;
    int get_col() const; 
    vector<float> get_point(char pointNumber) const;
    bool has_wall(char direction) const;
    void remove_wall(char direction);
    bool is_seen() const;
    void set_seen(bool value);

private:
    float row;
    float col;
    bool N;
    bool E;
    bool S;
    bool W;
    bool seen;
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class Sensor {
    public:
        float dist_measure;
        float offset_direction;
        float sens_dir;
        float offset_position;
        vector<float> sens_pos;

        Sensor();
        void init(vector<float> robot_pos, float robot_dir, float offset_direction_, float offset_position_);
        void updatePosition(vector<float> robot_pos, float robot_dir);
        void getDistanceToWall(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
};

class Robot {
    public:
        vector<float> rob_pos;
        vector<float> velocity;
        float rob_dir;
        float distance_wheels;
        float width;
        float height;
        Sensor sensR;
        Sensor sensR2;
        Sensor sensS;
        Sensor sensL2;
        Sensor sensL;

        Robot(float x, float y, float direction_, float distance_wheels, float width, float height);
        void updatePosition(float SpeedL, float SpeedR);
        void measureDistances(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
};

#endif
