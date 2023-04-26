#include <cmath>
#include <vector>
#include <stdio.h>
#include <iostream>
#include "estimations.h"
#include "defs.h"
#include "utils.h"
#include "classes.h"
#include <SDL2/SDL.h>

#define _USE_MATH_DEFINES

using namespace std;

float WALL_RETAIN = 0.9;
float POS_RETAIN = 0.999;
float SPEED_VAR = 0.4;

CellEst::CellEst() : N(0.5), E(0.5), S(0.5), W(0.5), seen(false) {}
void CellEst::initialize(float row_, float col_) {
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
int CellEst::get_row() const {return row;};
int CellEst::get_col() const {return col;};

vector<float> CellEst::get_point(char pointNumber) const {
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

bool CellEst::has_wall(char direction) const {
    switch (direction) {
        case 'N':
            if (N > 0.8) {return true;}
            else {return false;}
        case 'E':
            if (E > 0.8) {return true;}
            else {return false;}
        case 'S':
            if (S > 0.8) {return true;}
            else {return false;}
        case 'W':
            if (W > 0.8) {return true;}
            else {return false;}
        default:
            return true;
    }
}

void CellEst::update_wall(char direction, bool wallPresent) {
    switch (direction) {
        case 'N':
            N = WALL_RETAIN*N + (1-WALL_RETAIN)*wallPresent;
            break;
        case 'E':
            E = WALL_RETAIN*E + (1-WALL_RETAIN)*wallPresent;
            break;
        case 'S':
            S = WALL_RETAIN*S + (1-WALL_RETAIN)*wallPresent;
            break;
        case 'W':
            W = WALL_RETAIN*W + (1-WALL_RETAIN)*wallPresent;
            break;
    }
}

bool CellEst::is_seen() const {return seen;}

void CellEst::set_seen(bool value) {seen = value;}

SensorEst::SensorEst() {
    looking_at = '0';
    dist_measure = SENSOR_RANGE;
    intersection = {-1.0, -1.0};
    sens_pos.resize(2);
}

void SensorEst::init(vector<float> robot_pos, float robot_dir, float offset_direction_, float offset_position_) {
    dist_measure = SENSOR_RANGE;

    offset_direction = offset_direction_;
    sens_dir = robot_dir + offset_direction;

    offset_position = offset_position_;
    sens_pos[0] = robot_pos[0] + cos(sens_dir)*offset_position;
    sens_pos[1] = robot_pos[1] + sin(sens_dir)*offset_position;
}

void SensorEst::updatePosition(vector<float> robot_pos, float robot_dir) {
    sens_dir = robot_dir + offset_direction;

    sens_pos[0] = robot_pos[0] + cos(sens_dir)*this->offset_position;
    sens_pos[1] = robot_pos[1] + sin(sens_dir)*this->offset_position;
}

void SensorEst::getDistanceToWall(Sensor sensor) {
    dist_measure = sensor.dist_measure;
}

void SensorEst::compareDistanceToWall(SDL_Renderer *renderer, vector<float>& rob_pos, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    // Function to see if an intersection was expected and if yes to update the belief about a wall being there.
    looking_at = '0';
    intersection = {-1.0, -1.0};
    // float min_corner_dist = CELL_SIZE / 20;
    float min_corner_dist = 3;
    float tolerance_dist = 20;
    float ignore_max = 21;
    float shortest_dist_measure = SENSOR_RANGE;
    float temp_dist_measure = SENSOR_RANGE;
    int cell_column = -1;
    int cell_row = -1;
    bool intersection_found = false;
    vector<float> temp_intersection = {-1.0, -1.0};
    vector<float> end_sensor = {sens_pos[0] + cos(sens_dir) * SENSOR_RANGE, sens_pos[1] + sin(sens_dir) * SENSOR_RANGE};
    vector<float> wall_start;
    vector<float> wall_end;

    float p1;
    float p2;
    float p3;
    float p4;

    for (int i = 0; i < LABYRINTH_WIDTH; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1){
            wall_start = labyrinth[i][j].get_point('1');
            wall_end = labyrinth[i][j].get_point('2');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found && (temp_dist_measure < (SENSOR_RANGE-ignore_max))) {
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                        if ((distBetweenPoints(temp_intersection, wall_start) > min_corner_dist) && (distBetweenPoints(temp_intersection, wall_end) > min_corner_dist)){
                            looking_at = 'N';
                            cell_column = i;
                            cell_row = j;
                            intersection = temp_intersection;
                            p1 = wall_start[0];
                            p2 = wall_start[1];
                            p3 = wall_end[0];
                            p4 = wall_end[1];
                        }
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('N', false);
                    // labyrinth[i][j-1].update_wall('S', false); //TODO update opposite wall accordingly
                }
            }
            wall_start = labyrinth[i][j].get_point('2');
            wall_end = labyrinth[i][j].get_point('3');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found && (temp_dist_measure < (SENSOR_RANGE-ignore_max))) {
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                        if ((distBetweenPoints(temp_intersection, wall_start) > min_corner_dist) && (distBetweenPoints(temp_intersection, wall_end) > min_corner_dist)){
                            looking_at = 'E';
                            cell_column = i;
                            cell_row = j;
                            intersection = temp_intersection;
                            p1 = wall_start[0];
                            p2 = wall_start[1];
                            p3 = wall_end[0];
                            p4 = wall_end[1];
                        }
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('E', false);
                }
            }
            wall_start = labyrinth[i][j].get_point('3');
            wall_end = labyrinth[i][j].get_point('4');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found && (temp_dist_measure < (SENSOR_RANGE-ignore_max))) {   
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                        if ((distBetweenPoints(temp_intersection, wall_start) > min_corner_dist) && (distBetweenPoints(temp_intersection, wall_end) > min_corner_dist)){
                            looking_at = 'S';
                            cell_column = i;
                            cell_row = j;
                            intersection = temp_intersection;
                            p1 = wall_start[0];
                            p2 = wall_start[1];
                            p3 = wall_end[0];
                            p4 = wall_end[1];
                        }
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('S', false);
                }
            }
            wall_start = labyrinth[i][j].get_point('4');
            wall_end = labyrinth[i][j].get_point('1');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found && (temp_dist_measure < (SENSOR_RANGE-ignore_max))) {
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;
                        if ((distBetweenPoints(temp_intersection, wall_start) > min_corner_dist) && (distBetweenPoints(temp_intersection, wall_end) > min_corner_dist)){
                            looking_at = 'W';
                            cell_column = i;
                            cell_row = j;
                            intersection = temp_intersection;
                            p1 = wall_start[0];
                            p2 = wall_start[1];
                            p3 = wall_end[0];
                            p4 = wall_end[1];
                        }
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('W', false);
                }
            }
        }
    }
    if (looking_at != '0') {
        labyrinth[cell_column][cell_row].update_wall(looking_at, true);
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 128);
        SDL_RenderDrawLine(renderer, p1, p2, p3, p4);
        cout << "i: " << intersection[0] << "," << intersection[1] << endl;
        // if (labyrinth[cell_column][cell_row].has_wall(looking_at)){
        //     rob_pos[1] = POS_RETAIN*rob_pos[1] + (1-POS_RETAIN)*(intersection[1] - cos(sens_dir)*dist_measure);
        // }
    }
}

RobotEst::RobotEst(float x, float y, float direction_, float distance_wheels_, float width_, float height_) {
    rob_pos.resize(2);
    rob_pos = {x, y};
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

void RobotEst::get_offset_left(float& offset, float& theta_rel, float alpha=M_PI/4, float beta=M_PI/2){
    //   # alpha = angle between e1 and e2
    //   # beta = angle between e1 and forward direction (=e3)
    //   # get L 
    float dist_e1 = sensL.dist_measure + sensL.offset_position;
    float dist_e2 = sensL2.dist_measure + sensL2.offset_position;
    float L = sqrt( pow(dist_e2 * cos(alpha) - dist_e1, 2) + pow(dist_e2*sin(alpha), 2)) ;
    //   # Heron's Formula to get Area based on side lengths only
    float s = (dist_e1 + dist_e2 + L)/2;
    float A = sqrt( s * (s-dist_e1) * (s-dist_e2) * (s-L) );
    //   # use regular A = 1/2 * L * h (with h = offset)
    offset = (2*A)/L;

    //   # calculate theta
    float gamma = acos(max(min(offset/dist_e1, 1.0f), -1.0f));
    if (dist_e2*cos(alpha) < dist_e1){
        theta_rel = M_PI + beta - gamma;
    } else {
        theta_rel = M_PI + beta + gamma;
    }

  }

void RobotEst::get_offset_right(float& h, float& theta_rel, float alpha=M_PI/4, float beta=-M_PI/2){
    //   # alpha = angle between e4 and e5
    //   # beta = angle between e5 and forward direction (=e3)
    //   # get L 
    float dist_e4 = sensR2.dist_measure + sensR2.offset_position;
    float dist_e5 = sensR.dist_measure + sensR.offset_position;
    float L = sqrt( pow(dist_e4 * cos(alpha) - dist_e5, 2) + pow(-dist_e4*sin(alpha), 2) );
    //   # Heron's Formula to get Area based on side lengths only
    float s = (dist_e4 + dist_e5 + L)/2;
    float A = sqrt( s * (s-dist_e5) * (s-dist_e4) * (s-L) );
    //   # use regular A = 1/2 * L * h
    h = (2*A)/L;

    //   # calculate theta
    float gamma;
    gamma = acos(max(min(h/dist_e5, 1.0f), -1.0f));
    if (dist_e4*cos(alpha) < dist_e5){
        theta_rel = 2*M_PI + beta + gamma;
    } else {
        theta_rel = 2*M_PI + beta - gamma;
    }

    // theta_rel = M_PI + gamma;
    cout << "L: " << L << "  s: " << s << "  A: " << A << "  dist_e4: " << dist_e4 << "  dist_e5: " << dist_e5 << "  h: " << h << "   gamma: " << gamma << "   theta_rel: " << theta_rel << endl;
}

void RobotEst::get_offset_front_left(float& offset, float& theta_rel, float alpha=M_PI/4){
    float dist_e2 = sensL2.dist_measure + sensL2.offset_position;
    float dist_e3 = sensS.dist_measure + sensS.offset_position;
    float L = sqrt( pow(dist_e3 * cos(alpha) - dist_e2, 2) + pow(dist_e3*sin(alpha), 2) );
    float s = (dist_e2 + dist_e3 + L)/2;
    float A = sqrt( s*(s-dist_e2)*(s-dist_e3)*(s-L) );
    offset = (2*A)/L;
    theta_rel = 3*M_PI/2 - alpha + acos(max(min(offset/L, 1.0f), -1.0f));
}


void RobotEst::get_offset_front_right(float& offset, float& theta_rel, float alpha=M_PI/4){
    float dist_e3 = sensS.dist_measure + sensS.offset_position;
    float dist_e4 = sensR2.dist_measure + sensR2.offset_position;
    float L = sqrt( pow(dist_e3 * cos(alpha) - dist_e4, 2) + pow(dist_e3*sin(alpha), 2) );
    float s = (dist_e4 + dist_e3 + L)/2;
    float A = sqrt( s*(s-dist_e4)*(s-dist_e3)*(s-L) );
    offset = (2*A)/L;
    theta_rel = 2*M_PI - acos(max(min(offset/L, 1.0f), -1.0f)) - alpha;
}


void RobotEst::horizontal_dist_to_west_wall(float error_band=M_PI/6, float smoothing=0.1){
    char driving_dir;

    bool performed_update = false;
    float offset;
    float angle_rel;
    float xnew;
    float ynew;

    // cout << "----> R2:" << sensR2.looking_at << "   " << "   R:" << sensR.looking_at << "   pos: " << rob_pos[0] << endl;
    // cout << "intersection: " << sensR2.intersection[0] << "  " << "offset: " << offset << endl;

    //   # determine in which direction we are driving
    if (((2*M_PI - error_band) < rob_dir) || (rob_dir < error_band)) { //# yes, it is 'or'
        driving_dir = 'E';
    } else if ((3*M_PI/2 - error_band < rob_dir) && (rob_dir < 3*M_PI/2 + error_band)) {
        driving_dir = 'N';
    } else if ((M_PI - error_band < rob_dir) && (rob_dir < M_PI + error_band)) {
        driving_dir = 'W';
    } else if ((M_PI/2 - error_band < rob_dir) && (rob_dir < M_PI/2 + error_band)) {
        driving_dir = 'S';
    } else {
        // # if we are currently turning, don't update
        return;
    }

    // 1 & 2
    // if ((sensL.looking_at == sensL2.looking_at) && sensL.looking_at != '0') {
    //     performed_update = true;
    //     get_offset_left(offset, angle_rel, M_PI/4, M_PI/2);
    //     // # updating
    //     if((offset != offset) || (angle_rel != angle_rel)){
    //         return;
    //     }
    //     if (driving_dir == 'N'){
    //         xnew = sensL.intersection[0] + offset;
    //         rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * angle_rel;

    //         cout << "Wall to the left while moving North" << endl;
    //     }

    //     else if (driving_dir == 'S'){
    //         xnew = sensL.intersection[0] - offset;
    //         rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel + M_PI);

    //         cout << "Wall to the left while moving South" << endl;
    //     }

    //     else if (driving_dir == 'W'){
    //         ynew = sensL.intersection[1] - offset;
    //         rob_pos[1] = (1-smoothing) * rob_pos[1] + smoothing * ynew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - M_PI/2);

    //         cout << "Wall to the left while moving West" << endl; 
    //     }


    //     else if (driving_dir == 'E') {
    //         ynew = sensL.intersection[1] + offset;
    //         rob_pos[1] = (1-smoothing) * rob_pos[1] + smoothing * ynew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - 3*M_PI/2);

    //         cout << "Wall to the left while moving East" << endl;
    //     }
    // }

    // cout << "L2: " << sensL2.looking_at << "   S: " << sensS.looking_at << endl;

    // 2 & 3
    // if ((sensL2.looking_at == sensS.looking_at) && sensL2.looking_at != '0') {
    //     performed_update = true;
    //     get_offset_front_left(offset, angle_rel, M_PI/4);
    //     // # updating
    //     if((offset != offset) || (angle_rel != angle_rel)){
    //         cout << "NAN found!!!!!!!!!!!!!!!!!!!!!!!" << endl;
    //     }
    //     if (driving_dir == 'N'){
    //         ynew = sensS.intersection[1] + offset;
    //         rob_pos[1] = (1-smoothing) * rob_pos[1] + smoothing * ynew;
    //         cout << "before: " << rob_dir;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * angle_rel;
    //         cout << "   after: " << rob_dir << endl;
    //     }

    //     else if (driving_dir == 'S'){
    //         ynew = sensS.intersection[1] - offset;
    //         rob_pos[1] = (1-smoothing) * rob_pos[1] + smoothing * ynew;
    //         cout << "before: " << rob_dir;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - M_PI/2);
    //         cout << "   after: " << rob_dir << endl;
    //     }

    //     else if (driving_dir == 'W'){
    //         xnew = sensS.intersection[0] + offset;
    //         rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - M_PI/2);
    //     }

    //     else if (driving_dir == 'E') {
    //         xnew = sensS.intersection[0] - offset;
    //         rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - 3*M_PI/2);
    //     }
    // }

    // // 3 & 4
    // if (sensS.looking_at == sensR2.looking_at && sensS.looking_at != '0') {
    //     performed_update = true;
    //     get_offset_front_right(offset, angle_rel, M_PI/4);
    //     // # updating
    //     if((offset != offset) || (angle_rel != angle_rel)){
    //         return;
    //     }
    //     if (driving_dir == 'N'){
    //         ynew = sensS.intersection[1] + offset;
    //         rob_pos[1] = (1-smoothing) * rob_pos[1] + smoothing * ynew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * angle_rel;
    //     }

    //     else if (driving_dir == 'S'){
    //         ynew = sensS.intersection[1] - offset;
    //         rob_pos[1] = (1-smoothing) * rob_pos[1] + smoothing * ynew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - M_PI);
    //     }

    //     else if (driving_dir == 'W'){
    //         xnew = sensS.intersection[0] + offset;
    //         rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - M_PI/2);
    //     }

    //     else if (driving_dir == 'E') {12
    //         xnew = sensS.intersection[0] - offset;
    //         rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;
    //         rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - 3*M_PI/2);
    //     }
    // }

    rob_dir = fmod(rob_dir, 2*M_PI);
    if (rob_dir < 0) {
        rob_dir += 2*M_PI;
    }


    // cout << "wall_same: " << ((sensR2.looking_at == sensR.looking_at) && sensR2.looking_at != '0') << endl;

    // 4 & 5
    if ((sensR2.looking_at == sensR.looking_at) && sensR2.looking_at != '0'){
        performed_update = true;
        get_offset_right(offset, angle_rel, M_PI/4, -M_PI/2);
        // # updating
        if((offset != offset) || (angle_rel != angle_rel)){
            cout << "NAN found!!!!!!!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
        rob_dir = fmod(rob_dir, 2*M_PI);
        if (rob_dir < 0) {
            rob_dir += 2*M_PI;
        }
        cout << "before: " << rob_dir << endl;
        if (driving_dir == 'N'){
            xnew = sensR2.intersection[0] - offset;
            rob_pos[0] = (1-smoothing) * rob_pos[0] + smoothing * xnew;

            rob_dir = (1-smoothing) * rob_dir + smoothing * angle_rel;
            // cout << "sensR: " << sensR.looking_at << "   sensR2: " << sensR2.looking_at << endl;
            // cout << "Wall to the right while moving North" << endl;

        } else if (driving_dir == 'S') {
            xnew = sensR2.intersection[0] + offset;
            rob_pos[0] = ((1-smoothing) * rob_pos[0] + smoothing * xnew);
            rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel - M_PI);

            cout << "Wall to the right while moving South" << endl;

        } else if (driving_dir == 'W') {
            ynew = sensR2.intersection[1] + offset;
            rob_pos[1] = ((1-smoothing) * rob_pos[1] + smoothing * ynew);
            // cout << "before: " << rob_dir << endl;
            rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel + 3*M_PI/2);
            // cout << "after: " << rob_dir << "   sR: " << sensR.dist_measure << "   sR2: " << sensR2.dist_measure << endl;

            // cout << "Wall to the right while moving West" << endl;

        } else if (driving_dir == 'E') {
            ynew = sensR2.intersection[1] - offset;
            rob_pos[1] = ((1-smoothing) * rob_pos[1] + smoothing * ynew);
            rob_dir = (1-smoothing) * rob_dir + smoothing * (angle_rel + M_PI/2);

            // cout << "Wall to the right while moving East" << endl;
        }

        rob_dir = fmod(rob_dir, 2*M_PI);
        if (rob_dir < 0) {
            rob_dir += 2*M_PI;
        }   
        cout << "after: " << rob_dir << endl;
    }
}

// void RobotEst::estimatePos(Sensor sens1, Sensor sens2, Sensor sens3, Sensor sens4, Sensor sens5, float alpha) {
//     // get the length L of the wall segment 2 sensors are looking at
//     float L = sqrt( pow(-sens2.dist_measure * cos(alpha) + sens1.dist_measure, 2) + pow(sens2.dist_measure*sin(alpha), 2));
//     // Heron's Formula to get Area based on side lengths only
//     float s = sens1.dist_measure + sens2.dist_measure + L;
//     // Calculate Area
//     float A = sqrt( s * (s-sens1.dist_measure) * (s-sens2.dist_measure) * (s-L) );
//     // use regular A = 1/2 * L * h
//     float h = (2*A)/L;

//     //calculate theta
//     float gamma = acos(h/sens1.dist_measure);
//     float theta = 90 - gamma;
//     return h, theta;
// }

void RobotEst::updatePosition(float speedLeft_, float speedRight_) {
    float deltaTime = 1; //Param?
    float wheelRadius = 1;
    float speedLeft = speedLeft_ + gaussianNoise(0, SPEED_VAR);
    float speedRight = speedRight_ + + gaussianNoise(0, SPEED_VAR);
    // Calculate the new position of the robot
    float speed = wheelRadius * (speedLeft + speedRight) / 2;
    float angularVelocity = wheelRadius * (speedLeft - speedRight) / distance_wheels;

    // cout << "before: " << rob_dir;
    rob_dir = fmod(rob_dir + angularVelocity * deltaTime, 2*M_PI);
    if (rob_dir < 0) {
        rob_dir += 2*M_PI;
    }
    // cout << "   after: " << rob_dir << endl;
    rob_pos[0] += speed * cos(rob_dir) * deltaTime;
    rob_pos[1] += speed * sin(rob_dir) * deltaTime;

    // Calculate the new position of the sensors
    sensR.updatePosition(rob_pos, rob_dir);
    sensR2.updatePosition(rob_pos, rob_dir);
    sensS.updatePosition(rob_pos, rob_dir);
    sensL2.updatePosition(rob_pos, rob_dir);
    sensL.updatePosition(rob_pos, rob_dir);
    // cout << sensR.looking_at << " " << sensR.intersection[0] << "," << sensR.intersection[1] << endl;
}

void RobotEst::compareDistances(SDL_Renderer *renderer, Robot mouse, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    sensR.getDistanceToWall(mouse.sensR);
    sensR2.getDistanceToWall(mouse.sensR2);
    sensS.getDistanceToWall(mouse.sensS);
    sensL2.getDistanceToWall(mouse.sensL2);
    sensL.getDistanceToWall(mouse.sensL);

    sensR.compareDistanceToWall(renderer, rob_pos, labyrinth);
    sensR2.compareDistanceToWall(renderer, rob_pos, labyrinth);
    sensS.compareDistanceToWall(renderer, rob_pos, labyrinth);
    sensL2.compareDistanceToWall(renderer, rob_pos, labyrinth);
    sensL.compareDistanceToWall(renderer, rob_pos, labyrinth);
}