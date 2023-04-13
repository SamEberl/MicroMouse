#ifndef CLASSES_H
#define CLASSES_H

#include <vector>
#include "defs.h"

#include <SDL2/SDL.h>

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
        std::vector<float> position;
        std::vector<float> direction;
        std::vector<float> velocity;
        float speedForward;
        float distance_wheels;
        float width;
        float height;

        Robot(float x, float y, float x_dir, float y_dir, float distance_wheels, float width, float height);
        void updatePosition(SDL_Renderer *renderer, float SpeedL, float SpeedR);
        void updatePositionN(SDL_Renderer *renderer, float SpeedL, float SpeedR);
        void updatePositionNN(SDL_Renderer *renderer, float SpeedL, float SpeedR);
        void updatePositionO(SDL_Renderer *renderer, float SpeedL, float SpeedR);
        void updatePositionG(SDL_Renderer *renderer, float SpeedL, float SpeedR);
};

class Wall {
    public:
        bool seen;
        bool exists;
        std::vector<float> start;
        std::vector<float> end;

        Wall(std::vector<float> start_, std::vector<float> end_);
};


class Cell {
    public:
        std::vector<float> p1;
        std::vector<float> p2;
        std::vector<float> p3;
        std::vector<float> p4;

        bool north_seen;
        bool north_exists;
        bool east_seen;
        bool east_exists;
        bool south_seen;
        bool south_exists;
        bool west_seen;
        bool west_exists;


        Cell();
        Cell(float x_id, float y_id);
        void initialize(float x_id, float y_id);
        void updateWall(char wall_direction, bool wall_exists);
};

class Sensor {
    public:
        float offset_direction;
        float offset_position;
        float radius;
        float dist_measure;
        std::vector<float> position;
        std::vector<float> direction;

        Sensor(Robot mouse, float offset_direction_, float offset_position_);
        void updatePosition(Robot mouse);
        void getDistance(Cell[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
};

#endif
