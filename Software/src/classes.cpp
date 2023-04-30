#include <cmath>
#include <vector>
#include <stdio.h>
#include <iostream>
#include "classes.h"
#include "defs.h"
#include "utils.h"
#include <SDL2/SDL.h>

#define _USE_MATH_DEFINES

// use array<float, 2> instead of vector for efficiency?

Corner::Corner() {}
void Corner::initialize(float row_, float col_) {
    row = row_;
    col = col_;
    float x_pos = col * (CELL_SIZE + WALL_WIDTH);
    float y_pos = row * (CELL_SIZE + WALL_WIDTH);
    // Points start in top left corner of cell and go clockwise
    this->p1 = {x_pos, y_pos};
    this->p2 = {(x_pos+WALL_WIDTH), y_pos};
    this->p3 = {(x_pos+WALL_WIDTH), (y_pos+WALL_WIDTH)};
    this->p4 = {x_pos, (y_pos+WALL_WIDTH)};
}
int Corner::get_row() const {return row;};
int Corner::get_col() const {return col;};
vector<float> Corner::get_point(char pointNumber) const {
    switch (pointNumber) {
        case '1':
            return p1;
        case '2':
            return p2;
        case '3':
            return p3;
        case '4':
            return p4;
        default:
            throw invalid_argument("Invalid point number");
    }
}

Cell::Cell() : N(true), E(true), S(true), W(true), seen(false) {}
void Cell::initialize(float row_, float col_) {
    row = row_;
    col = col_;
    float x_pos = col * (CELL_SIZE + WALL_WIDTH) + WALL_WIDTH;
    float y_pos = row * (CELL_SIZE + WALL_WIDTH) + WALL_WIDTH;
    // Points start in top left corner of cell and go clockwise
    this->p1 = {x_pos, y_pos};
    this->p2 = {(x_pos+CELL_SIZE), y_pos};
    this->p3 = {(x_pos+CELL_SIZE), (y_pos+CELL_SIZE)};
    this->p4 = {x_pos, (y_pos+CELL_SIZE)};
}
int Cell::get_row() const {return row;};
int Cell::get_col() const {return col;};

vector<float> Cell::get_point(char pointNumber) const {
    switch (pointNumber) {
        case '1':
            return p1;
        case '2':
            return p2;
        case '3':
            return p3;
        case '4':
            return p4;
        default:
            throw invalid_argument("Invalid point number");
    }
}

bool Cell::has_wall(char direction) const {
    switch (direction) {
        case 'N':
            return N;
        case 'E':
            return E;
        case 'S':
            return S;
        case 'W':
            return W;
        default:
            return true;
    }
}

void Cell::remove_wall(char direction) {
    switch (direction) {
        case 'N':
            N = false;
            break;
        case 'E':
            E = false;
            break;
        case 'S':
            S = false;
            break;
        case 'W':
            W = false;
            break;
    }
}

bool Cell::is_seen() const {return seen;}

void Cell::set_seen(bool value) {seen = value;}

Sensor::Sensor() {
    dist_measure = SENSOR_RANGE;
    sens_pos.resize(2);
}

void Sensor::init(vector<float> robot_pos, float robot_dir, float offset_direction_, float offset_position_) {
    dist_measure = SENSOR_RANGE;

    offset_direction = offset_direction_;
    sens_dir = robot_dir + offset_direction;

    offset_position = offset_position_;
    sens_pos[0] = robot_pos[0] + cos(sens_dir)*offset_position;
    sens_pos[1] = robot_pos[1] + sin(sens_dir)*offset_position;
}

void Sensor::updatePosition(vector<float> robot_pos, float robot_dir) {
    sens_dir = robot_dir + offset_direction;

    sens_pos[0] = robot_pos[0] + cos(sens_dir)*this->offset_position;
    sens_pos[1] = robot_pos[1] + sin(sens_dir)*this->offset_position;
}

void Sensor::getDistanceToWall(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    float shortest_dist_measure = SENSOR_RANGE;
    float temp_dist_measure = SENSOR_RANGE;
    bool intersection_found = false;
    vector<float> intersection = {-1.0, -1.0};
    vector<float> end_sensor = {sens_pos[0] + cos(sens_dir) * SENSOR_RANGE, sens_pos[1] + sin(sens_dir) * SENSOR_RANGE};
    vector<float> wall_start;
    vector<float> wall_end;
    for (int i = 0; i < LABYRINTH_WIDTH; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1){
            if (labyrinth[i][j].has_wall('N')){
                wall_start = labyrinth[i][j].get_point('1');
                wall_end = labyrinth[i][j].get_point('2');
                intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].has_wall('E')){
                wall_start = labyrinth[i][j].get_point('2');
                wall_end = labyrinth[i][j].get_point('3');
                intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].has_wall('S')){
                wall_start = labyrinth[i][j].get_point('3');
                wall_end = labyrinth[i][j].get_point('4');
                intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].has_wall('W')){
                wall_start = labyrinth[i][j].get_point('4');
                wall_end = labyrinth[i][j].get_point('1');
                intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
        }
    }

    for (int i = 0; i < LABYRINTH_WIDTH+1; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT+1; j += 1){
            Corner test_corner = corners[i][j];
            wall_start = corners[i][j].get_point('1');
            wall_end = corners[i][j].get_point('2');
            intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (shortest_dist_measure > temp_dist_measure) {
                    shortest_dist_measure = temp_dist_measure;
                }
            }
            wall_start = corners[i][j].get_point('2');
            wall_end = corners[i][j].get_point('3');
            intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (shortest_dist_measure > temp_dist_measure) {
                    shortest_dist_measure = temp_dist_measure;
                }
            }
            wall_start = corners[i][j].get_point('3');
            wall_end = corners[i][j].get_point('4');
            intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (shortest_dist_measure > temp_dist_measure) {
                    shortest_dist_measure = temp_dist_measure;
                }
            }
            wall_start = corners[i][j].get_point('4');
            wall_end = corners[i][j].get_point('1');
            intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (shortest_dist_measure > temp_dist_measure) {
                    shortest_dist_measure = temp_dist_measure;
                }
            }
        }
    }

    dist_measure = shortest_dist_measure + gaussianNoise(0, 0);
}


Robot::Robot(float x, float y, float direction_, float distance_wheels_, float width_, float height_) {
    rob_pos.resize(2);
    rob_pos = {x, y};
    velocity = {x, y};
    rob_dir = direction_;
    distance_wheels = distance_wheels_;
    width = width_;
    height = height_;

    sensR.init(rob_pos, rob_dir, M_PI/2, width/2);
    sensR2.init(rob_pos, rob_dir, M_PI/4, height/2);
    sensS.init(rob_pos, rob_dir, 0.0, height/2);
    sensL2.init(rob_pos, rob_dir, -M_PI/4, height/2);
    sensL.init(rob_pos, rob_dir, -M_PI/2, width/2);
}

void Robot::updatePosition(float speedLeft, float speedRight) {
    float deltaTime = 1; //Param?
    float wheelRadius = 1;
    // Calculate the new position of the robot
    float speed = wheelRadius * (speedLeft + speedRight) / 2;
    float angularVelocity = wheelRadius * (speedLeft - speedRight) / distance_wheels;
    rob_dir = fmod(rob_dir + angularVelocity * deltaTime, 2*M_PI);
    if (rob_dir < 0) {
        rob_dir += 2*M_PI;
    }
    rob_pos[0] += speed * cos(rob_dir) * deltaTime;
    rob_pos[1] += speed * sin(rob_dir) * deltaTime;

    // Calculate the new position of the sensors
    sensR.updatePosition(rob_pos, rob_dir);
    sensR2.updatePosition(rob_pos, rob_dir);
    sensS.updatePosition(rob_pos, rob_dir);
    sensL2.updatePosition(rob_pos, rob_dir);
    sensL.updatePosition(rob_pos, rob_dir);
}

void Robot::measureDistances(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    sensR.getDistanceToWall(labyrinth, corners);
    sensR2.getDistanceToWall(labyrinth, corners);
    sensS.getDistanceToWall(labyrinth, corners);
    sensL2.getDistanceToWall(labyrinth, corners);
    sensL.getDistanceToWall(labyrinth, corners);
}
