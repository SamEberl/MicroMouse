#include <cmath>
#include <vector>
#include <stdio.h>
#include "classes.h"

#define _USE_MATH_DEFINES

Robot::Robot(float x, float y, float x_dir, float y_dir, float radius_) {
    position.resize(2);
    direction.resize(2);
    position = {x, y};
    direction = {x_dir, y_dir};
    radius = radius_;
}

void Robot::updatePosition(float SpeedL, float SpeedR) {
    speedForward = std::min(SpeedL, SpeedR);
    // theta is the angle by which the robot is turned by in radians (pi = 180°)
    float theta;
    if (SpeedR <= SpeedL){
        theta = std::abs(SpeedL-SpeedR)/radius;
    } else {
        theta = (2*M_PI) - (std::abs(SpeedL-SpeedR)/radius);
    }

    // update position considering same speed between both wheels and difference between wheels
    position[0] += (direction[0] * speedForward) + (direction[0] * (1-cos(theta)) / 2);
    position[1] += (direction[1] * speedForward) + (direction[1] * (sin(theta)) / 2);

    //get magnitude of direction
    float magnitude = sqrt((direction[0]*direction[0]) + (direction[1]*direction[1]));

    //update the direction in which the robot is facing and normalize by magnitude
    direction[0] = (cos(theta)*direction[0] - sin(theta)*direction[1])/magnitude;
    direction[1] = (sin(theta)*direction[0] + cos(theta)*direction[1])/magnitude;
}