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


Cell::Cell(){
    north_seen = true;
    north_exists = true;
    east_seen = true;
    east_exists = true;
    south_seen = true;
    south_exists = true;
    west_seen = true;
    west_exists = true;
}

void Cell::initialize(float x_id, float y_id) {
    // Points start in top left corner of cell and go clockwise
    this->p1 = {x_id, y_id};
    this->p2 = {(x_id+1), y_id};
    this->p3 = {(x_id+1), (y_id+1)};
    this->p4 = {x_id, (y_id+1)};
}


Sensor::Sensor(Robot mouse, float offset_direction_, float offset_position_) {
    position.resize(2);
    offset_direction = offset_direction_;
    offset_position = offset_position_;
    dist_measure = SENSOR_RANGE;
    position[0] = mouse.position[0] + cos(mouse.direction + offset_direction)*offset_position;
    position[1] = mouse.position[1] + sin(mouse.direction + offset_direction)*offset_position;
}

void Sensor::updatePosition(Robot mouse) {
    position[0] = mouse.position[0] + cos(mouse.direction + offset_direction)*this->offset_position;
    position[1] = mouse.position[1] + sin(mouse.direction + offset_direction)*this->offset_position;
}

void Sensor::getDistance(Robot mouse, Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    float shortest_dist_measure = SENSOR_RANGE;
    float temp_dist_measure = SENSOR_RANGE;
    std::vector<float> end_sensor = {0, 0};
    for (int i = 0; i < LABYRINTH_WIDTH; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1){
            end_sensor[0] = position[0] + cos(mouse.direction + offset_direction)*SENSOR_RANGE;
            end_sensor[1] = position[1] + sin(mouse.direction + offset_direction)*SENSOR_RANGE;
            if (labyrinth[i][j].north_seen){
                if (labyrinth[i][j].north_exists){
                    temp_dist_measure = doIntersect(position, end_sensor, labyrinth[i][j].p1, labyrinth[i][j].p2);
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].east_seen){
                if (labyrinth[i][j].east_exists){
                    temp_dist_measure = doIntersect(position, end_sensor, labyrinth[i][j].p2, labyrinth[i][j].p3);
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
            if (labyrinth[i][j].south_seen){
                if (labyrinth[i][j].south_exists){
                    temp_dist_measure = doIntersect(position, end_sensor, labyrinth[i][j].p3, labyrinth[i][j].p4);
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    } 
                }
            }
            if (labyrinth[i][j].west_seen){
                if (labyrinth[i][j].west_exists){
                    temp_dist_measure = doIntersect(position, end_sensor, labyrinth[i][j].p4, labyrinth[i][j].p1);
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                    }
                }
            }
        }
    }
    dist_measure = shortest_dist_measure;
}


/*
    Wall north = Wall(p1, p2);
    Wall east = Wall(p2, p4);
    Wall south = Wall(p3, p4);
    Wall west = Wall(p1, p3);


Wall::Wall(std::vector<float> start_, std::vector<float> end_) {
    seen = true;
    exists = true;
    start = start_;
    end = end_; 
}
*/





/*
void Cell::updateWall(char wall_direction, bool wall_exists){
    switch(wall_direction){
        case 'n':
            north_seen = true;
            north_exists = wall_exists;
            break;

        case 'e':
            north_seen = true;
            north_exists = wall_exists;
            break;

        case 's':
            north_seen = true;
            north_exists = wall_exists;
            break;

        case 'w':
            north_seen = true;
            north_exists = wall_exists;
            break;

        default:
            printf("Error! Passed parameter 'wall_direction' incorrect.");
    }
}
*/




