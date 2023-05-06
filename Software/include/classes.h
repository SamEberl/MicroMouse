#ifndef CLASSES_H
#define CLASSES_H

#include <vector>
#include "defs.h"
#include <SDL2/SDL.h>

using namespace std;

class Corner {
public:
    Corner();
    void initialize(float row, float col);
    int get_row() const;
    int get_col() const; 
    vector<float> get_point(char pointNumber) const;

private:
    float row;
    float col;
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class Cell {
public:
    Cell();
    void initialize(float row, float col);
    void remove_wall(char direction);
    void set_seen(bool value);
    bool is_seen() const;
    bool has_wall(char direction) const;
    int get_row() const;
    int get_col() const; 
    vector<float> get_point(char pointNumber) const;

    bool seen;

private:
    float row;
    float col;
    bool N;
    bool E;
    bool S;
    bool W;
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class Sensor {
    public:
        Sensor();
        void init(vector<float> robot_pos, float robot_dir, float offset_direction_, float offset_position_);
        void updatePosition(vector<float> robot_pos, float robot_dir);
        void getDistanceToWall(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);

        float dist_measure;
        float offset_direction;
        float sens_dir;
        float offset_position;
        vector<float> sens_pos;
};

class Robot {
    public:
        Robot(float x, float y, float direction_, float distance_wheels, float width, float height);
        void updatePosition(float SpeedL, float SpeedR);
        void measureDistances(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);

        float rob_dir;
        float distance_wheels;
        float width;
        float height;
        float speedLeft;
        float speedRight;
        Sensor sensR;
        Sensor sensR2;
        Sensor sensS;
        Sensor sensL2;
        Sensor sensL;
        vector<float> rob_pos;
        vector<vector<float>> taken_path;

};

#endif
