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
    direction.resize(2);
    position = {x, y};
    direction = {x_dir, y_dir};
    velocity = {x, y};
    speedForward = 0;
    distance_wheels = distance_wheels_;
    width = width_;
    height = height_;
}


void Robot::updatePositionNN(SDL_Renderer *renderer, float SpeedL, float SpeedR) {
    // Calculate the speed of the robot
    speedForward = (SpeedL + SpeedR) / 2;
    // Calculate the angular velocity of the robot
    float angularVelocity = (SpeedR - SpeedL) / distance_wheels;
    // Calculate the new direction of the robot
    direction[0] = direction[0] + angularVelocity;
    // Calculate the new position of the robot
    position[0] = position[0] + speedForward * cos(direction[0]);
    position[1] = position[1] + speedForward * sin(direction[0]);
}


void Robot::updatePositionN(SDL_Renderer *renderer, float SpeedL, float SpeedR) {
    std::vector<float> wheel_left(2), wheel_right(2);
    std::vector<float> base_left(2), base_right(2);

    float alpha = atan2(direction[1], direction[0]);
    float ca = cos(alpha);
    float sa = sin(alpha);
    float ca_Ninety = cos(alpha + M_PI_2);
    float sa_Ninety = sin(alpha + M_PI_2);

    float cNinety = cos(M_PI_2);
    float sNinety = sin(M_PI_2);

    wheel_left[0] = -ca_Ninety*distance_wheels + sa_Ninety*SpeedL + position[0];
    wheel_left[1] = sa_Ninety*distance_wheels + ca_Ninety*SpeedL + position[1];
    wheel_right[0] = ca_Ninety*distance_wheels + sa_Ninety*SpeedR + position[0];
    wheel_right[1] = -sa_Ninety*distance_wheels + ca_Ninety*SpeedR + position[1];
    
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    //SDL_RenderDrawPoint(renderer, wheel_left[0], wheel_left[1]);
    SDL_RenderDrawPoint(renderer, wheel_right[0], wheel_right[1]);
    

    base_left[0] = cNinety*direction[0] - sNinety*direction[1] + position[0];
    base_left[1] = sNinety*direction[0] + cNinety*direction[1] + position[1];
    base_right[0] = cNinety*direction[0] + sNinety*direction[1] + position[0];
    base_right[1] = -sNinety*direction[0] + cNinety*direction[1] + position[1];
    

    std::vector<float> intersect = lineLineIntersection(wheel_left, wheel_right, base_left, base_right);

    float radius = distBetweenPoints(intersect, position);
    //what if intersect == position?
    //what if SpeedL == SpeedR?


    float beta = atan2(abs(SpeedL-SpeedR), distance_wheels*2);
    float cb = cos(beta);
    float sb = sin(beta);

    std::cout << "before " << "x: " << position[0] << "  y: " << position[1] << std::endl;
    std::cout << "intersect: " << intersect.at(0) << "  intersect: " << intersect.at(1) << std::endl;
    position[0] =   cb*(position[0]-intersect[0]) + sb*(position[1]-intersect[1]) + intersect[0];
    position[1] = - sb*(position[0]-intersect[0]) + cb*(position[1]-intersect[1]) + intersect[1];
    std::cout << "after " << "x: " << position[0] << "  y: " << position[1] << std::endl;
    // std::cout << "1:" << cb*(position[0]-intersect[0]) << "  2:" << - sb*(position[0]-intersect[0]) << "  3:" << intersect[0] << std::endl;
    // std::cout << "1:" << sb*(position[1]-intersect[1]) << "  2:" << cb*(position[1]-intersect[1]) << "  3:" << intersect[1] << std::endl;
    

    //get magnitude of direction
    float magnitude = sqrt((direction[0]*direction[0]) + (direction[1]*direction[1]));

    //update the direction in which the robot is facing and normalize by magnitude
    direction[0] = ( cb*direction[0] + sb*direction[1])/magnitude;
    direction[1] = (- sb*direction[0] + cb*direction[1])/magnitude;
    std::cout << "dir: " << direction[0] << "  " << direction[1] << std::endl;
}

void Robot::updatePosition(SDL_Renderer *renderer, float SpeedL, float SpeedR) {
    // Calculate the angular velocity of the robot
    float angularVelocity = (SpeedR - SpeedL) / distance_wheels;
    // Calculate the new direction of the robot
    direction[0] += angularVelocity;
    // Set the speed of the robot to zero
    speedForward = 0;
    // Calculate the new position of the robot
    position[0] += speedForward * cos(direction[0]);
    position[1] += speedForward * sin(direction[0]);
    // Calculate the new velocity of the robot
    velocity[0] = speedForward * cos(direction[0]);
    velocity[1] = speedForward * sin(direction[0]);
    // Render the robot at the new position

}


void Robot::updatePositionG(SDL_Renderer *renderer, float SpeedL, float SpeedR) {
    // Calculate the speed of the robot
    speedForward = (SpeedL + SpeedR) / 2;
    // Calculate the angular velocity of the robot
    float angularVelocity = (SpeedR - SpeedL) / distance_wheels;
    // Calculate the new direction of the robot
    direction[0] = direction[0] + angularVelocity;
    // Calculate the new position of the robot
    position[0] = position[0] + speedForward * cos(direction[0]);
    position[1] = position[1] + speedForward * sin(direction[0]);
    // Calculate the new velocity of the robot
    velocity[0] = speedForward * cos(direction[0]);
    velocity[1] = speedForward * sin(direction[0]);
    // Render the robot at the new position
}


void Robot::updatePositionO(SDL_Renderer *renderer, float SpeedL, float SpeedR) {
    //TODO: Put in delay from when new Speed arrives
    speedForward = std::min(SpeedL, SpeedR);
    // alpha is the angle by which the robot is turned by in radians (pi = 180°)
    float alpha;
    if (SpeedR <= SpeedL){
        alpha = std::abs(SpeedL-SpeedR)/distance_wheels;
    } else {
        alpha = (2*M_PI) - (std::abs(SpeedL-SpeedR)/distance_wheels);
    }

    // update position considering same speed between both wheels and difference between wheels
    position[0] += (direction[0] * speedForward) + (direction[0] * (1-cos(alpha)) / 2);
    position[1] += (direction[1] * speedForward) + (direction[1] * (sin(alpha)) / 2);

    //get magnitude of direction
    float magnitude = sqrt((direction[0]*direction[0]) + (direction[1]*direction[1]));

    //update the direction in which the robot is facing and normalize by magnitude
    direction[0] = (cos(alpha)*direction[0] - sin(alpha)*direction[1])/magnitude;
    direction[1] = (sin(alpha)*direction[0] + cos(alpha)*direction[1])/magnitude;
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
    direction.resize(2);
    position.resize(2);
    offset_direction = offset_direction_;
    offset_position = offset_position_;
    dist_measure = SENSOR_RANGE;
    direction[0] = (cos(offset_direction)*mouse.direction[0] - sin(offset_direction)*mouse.direction[1]);
    direction[1] = (sin(offset_direction)*mouse.direction[0] + cos(offset_direction)*mouse.direction[1]);
    position[0] = mouse.position[0] + direction[0]*offset_position;
    position[1] = mouse.position[1] + direction[1]*offset_position;
}

void Sensor::updatePosition(Robot mouse) {
    direction[0] = (cos(this->offset_direction)*mouse.direction[0] - sin(this->offset_direction)*mouse.direction[1]);
    direction[1] = (sin(this->offset_direction)*mouse.direction[0] + cos(this->offset_direction)*mouse.direction[1]);
    position[0] = mouse.position[0] + direction[0]*this->offset_position;
    position[1] = mouse.position[1] + direction[1]*this->offset_position;
}

void Sensor::getDistance(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    float shortest_dist_measure = SENSOR_RANGE;
    float temp_dist_measure = SENSOR_RANGE;
    std::vector<float> end_sensor = {0, 0};
    for (int i = 0; i < LABYRINTH_WIDTH; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1){
            end_sensor[0] = position[0] + direction[0]*SENSOR_RANGE;
            end_sensor[1] = position[1] + direction[1]*SENSOR_RANGE;
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




