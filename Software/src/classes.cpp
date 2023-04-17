#include <cmath>
#include <vector>
#include <stdio.h>
#include <iostream>
#include "classes.h"
#include "defs.h"
#include "utils.h"

#include <SDL2/SDL.h>

#define _USE_MATH_DEFINES


Robot::Robot(float x, float y, float x_dir, float y_dir, float distance_wheels_, float width_, float height_) {
    position.resize(2);
    position = {x, y};
    velocity = {x, y};
    direction = -M_PI_2;
    distance_wheels = distance_wheels_;
    width = width_;
    height = height_;
}


void Robot::updatePosition(float speedLeft, float speedRight) {
    // Calculate the speed of the robot
    float speed = (speedLeft + speedRight) / 2;
    // Calculate the angular velocity of the robot
    float angularVelocity = (speedLeft - speedRight) / distance_wheels;
    // Update the direction of the robot
    float deltaTime = 1; //Param?
    direction += angularVelocity * deltaTime;
    // Calculate the new position of the robot
    position[0] += speed * cos(direction) * deltaTime;
    position[1] += speed * sin(direction) * deltaTime;
}

Cell::Cell() : N(true), E(true), S(true), W(true), visited(false) {}
void Cell::initialize(float row_, float col_) {
    row = row_;
    col = col_;
    float x_pos = col * CELL_SIZE;
    float y_pos = row * CELL_SIZE;
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

bool Cell::is_visited() const {return visited;}

void Cell::set_visited(bool value) {visited = value;}


Sensor::Sensor(Robot mouse, float offset_direction_, float offset_position_) {
    dist_measure = SENSOR_RANGE;

    offset_direction = offset_direction_;
    direction = mouse.direction + offset_direction;

    position.resize(2);
    offset_position = offset_position_;
    position[0] = mouse.position[0] + cos(direction)*offset_position;
    position[1] = mouse.position[1] + sin(direction)*offset_position;


}

void Sensor::updatePosition(Robot mouse) {
    direction = mouse.direction + offset_direction;

    position[0] = mouse.position[0] + cos(direction)*this->offset_position;
    position[1] = mouse.position[1] + sin(direction)*this->offset_position;
}

void Sensor::getDistanceToWall(SDL_Renderer *renderer, Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    float shortest_dist_measure = SENSOR_RANGE;
    float temp_dist_measure = SENSOR_RANGE;
    bool intersection_found = false;
    vector<float> intersection = {-1.0, -1.0};
    vector<float> end_sensor = {position[0] + cos(direction) * SENSOR_RANGE, position[1] + sin(direction) * SENSOR_RANGE};
    vector<float> wall_start;
    vector<float> wall_end;
    for (int i = 0; i < LABYRINTH_WIDTH; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1){
            if (labyrinth[i][j].has_wall('N')){
                wall_start = labyrinth[i][j].get_point('1');
                wall_end = labyrinth[i][j].get_point('2');
                intersection = findIntersection(renderer,
                                                position, 
                                                end_sensor, 
                                                wall_start, 
                                                wall_end, 
                                                temp_dist_measure, 
                                                intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].has_wall('E')){
                wall_start = labyrinth[i][j].get_point('2');
                wall_end = labyrinth[i][j].get_point('3');
                intersection = findIntersection(renderer,
                                                position, 
                                                end_sensor, 
                                                wall_start, 
                                                wall_end, 
                                                temp_dist_measure, 
                                                intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].has_wall('S')){
                wall_start = labyrinth[i][j].get_point('3');
                wall_end = labyrinth[i][j].get_point('4');
                intersection = findIntersection(renderer,
                                                position, 
                                                end_sensor, 
                                                wall_start, 
                                                wall_end, 
                                                temp_dist_measure, 
                                                intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].has_wall('W')){
                wall_start = labyrinth[i][j].get_point('4');
                wall_end = labyrinth[i][j].get_point('1');
                intersection = findIntersection(renderer,
                                                position, 
                                                end_sensor, 
                                                wall_start, 
                                                wall_end, 
                                                temp_dist_measure, 
                                                intersection_found);
                if (intersection_found){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
        }
    }
    dist_measure = shortest_dist_measure;
}
