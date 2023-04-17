#ifndef CLASSES_H
#define CLASSES_H

#include <vector>
#include "defs.h"

#include <SDL2/SDL.h>

using namespace std;

struct Point {
    float x;
    float y;
};

/*
class LineSegment {
public:
    float start, end;

    LineSegment(Point start, Point end);
};
*/


class Robot {
    public:
        vector<float> position;
        vector<float> velocity;
        float direction;
        float distance_wheels;
        float width;
        float height;

        Robot(float x, float y, float x_dir, float y_dir, float distance_wheels, float width, float height);
        void updatePosition(float SpeedL, float SpeedR);
};

class Wall {
    public:
        bool seen;
        bool exists;
        vector<float> start;
        vector<float> end;

        Wall(vector<float> start_, vector<float> end_);
};


class Cell {
public:
    Cell();
    void initialize(float row, float col);
    int get_row() const;
    int get_col() const; 
    vector<float> get_point(char pointNumber) const;
    bool has_wall(char direction) const;
    void remove_wall(char direction);
    bool is_visited() const;
    void set_visited(bool value);

private:
    float row;
    float col;
    bool N;
    bool E;
    bool S;
    bool W;
    bool visited;
    vector<float> p1;
    vector<float> p2;
    vector<float> p3;
    vector<float> p4;
};

class Sensor {
    public:
        float dist_measure;
        float offset_direction;
        float direction;
        float offset_position;
        vector<float> position;

        Sensor(Robot mouse, float offset_direction_, float offset_position_);
        void updatePosition(Robot mouse);
        void getDistance(Robot mouse, Cell[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
        void getDistanceToWall(SDL_Renderer *renderer, Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
};

#endif
