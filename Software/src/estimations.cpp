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

float WALL_RETAIN = 0.99;
float POS_RETAIN = 0.999;
float SENS_VAR = 5.0;
float ENCODER_VAR = 0.1;

CornerEst::CornerEst() {}
void CornerEst::initialize(float row, float col) {
    float x_pos = col * (CELL_SIZE + WALL_WIDTH);
    float y_pos = row * (CELL_SIZE + WALL_WIDTH);
    // Points start in top left corner of cell and go clockwise
    this->p1 = {x_pos, y_pos};
    this->p2 = {(x_pos+WALL_WIDTH), y_pos};
    this->p3 = {(x_pos+WALL_WIDTH), (y_pos+WALL_WIDTH)};
    this->p4 = {x_pos, (y_pos+WALL_WIDTH)};
}
vector<float> CornerEst::get_point(char pointNumber) const {
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

CellEst::CellEst() : N(0.5), E(0.5), S(0.5), W(0.5), seen(false), is_goal(false) {}
void CellEst::initialize(float row, float col) {
    float x_pos = col * (CELL_SIZE + WALL_WIDTH) + WALL_WIDTH;
    float y_pos = row * (CELL_SIZE + WALL_WIDTH) + WALL_WIDTH;
    // Points start in top left corner of cell and go clockwise
    this->p1 = {x_pos, y_pos};
    this->p2 = {(x_pos+CELL_SIZE), y_pos};
    this->p3 = {(x_pos+CELL_SIZE), (y_pos+CELL_SIZE)};
    this->p4 = {x_pos, (y_pos+CELL_SIZE)};
    if (row == 0) {
        N = 1.0;
    }
    if (col == LABYRINTH_WIDTH-1) {
        E = 1.0;
    }
    if (row == LABYRINTH_HEIGHT-1) {
        S = 1.0;
    }
    if (col == 0) {
        W = 1.0;
    }
}

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
    float threshold_seen = 0.6;
    switch (direction) {
        case 'N':
            if (N > threshold_seen) {return true;}
            else {return false;}
        case 'E':
            if (E > threshold_seen) {return true;}
            else {return false;}
        case 'S':
            if (S > threshold_seen) {return true;}
            else {return false;}
        case 'W':
            if (W > threshold_seen) {return true;}
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

// void CellEst::update_wall_seen(char direction) {
//         switch (direction) {
//         case 'N':
//             if (N > threshold_seen) {this->seen = true;}
//             else if (N < (1-threshold_seen)) {this->seen = true;}
//             else {this->seen = false;}
//             break;
//         case 'E':
//             if (E > threshold_seen) {this->seen = true;}
//             else if (E < (1-threshold_seen)) {this->seen = true;}
//             else {this->seen = false;}
//             break;
//         case 'S':
//             S = WALL_RETAIN*S + (1-WALL_RETAIN)*wallPresent;
//             break;
//         case 'W':
//             W = WALL_RETAIN*W + (1-WALL_RETAIN)*wallPresent;
//             break;

// }

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
    dist_measure = sensor.dist_measure + gaussianNoise(0, SENS_VAR);
}

void SensorEst::compareDistanceToWall(SDL_Renderer *renderer, vector<float>& rob_pos, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    // Function to see if an intersection was expected and if yes to update the belief about a wall being there.
    looking_at = '0';
    intersection = {-1.0, -1.0};
    // float min_corner_dist = CELL_SIZE / 20;
    float min_corner_dist = 20;
    float tolerance_dist = 20;
    float ignore_max = 20;
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
            if (intersection_found) {
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (dist_measure < (SENSOR_RANGE-ignore_max)){
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
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('N', false);
                    if (i-1 >= 0){
                        labyrinth[i-1][j].update_wall('S', false);
                    }
                }
            }
            wall_start = labyrinth[i][j].get_point('2');
            wall_end = labyrinth[i][j].get_point('3');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found) {
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (dist_measure < (SENSOR_RANGE-ignore_max)){
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
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('E', false);
                    if (j+1 < LABYRINTH_WIDTH){
                        labyrinth[i][j+1].update_wall('W', false);
                    }
                }
            }
            wall_start = labyrinth[i][j].get_point('3');
            wall_end = labyrinth[i][j].get_point('4');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found) {   
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (dist_measure < (SENSOR_RANGE-ignore_max)){
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
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('S', false);
                    if (i+1 < LABYRINTH_HEIGHT){
                        labyrinth[i+1][j].update_wall('N', false);
                    }
                }
            }
            wall_start = labyrinth[i][j].get_point('4');
            wall_end = labyrinth[i][j].get_point('1');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found) {
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (dist_measure < (SENSOR_RANGE-ignore_max)){
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
                    } 
                } else if (dist_measure > (temp_dist_measure + tolerance_dist)) {
                    labyrinth[i][j].update_wall('W', false);
                    if (j-1 >= 0){
                        labyrinth[i][j-1].update_wall('E', false);
                    }
                }
            }
        }
    }

    // Check intersection with corners
    for (int i = 0; i < LABYRINTH_WIDTH+1; i += 1){
        for (int j = 0; j < LABYRINTH_HEIGHT+1; j += 1){
            wall_start = corners[i][j].get_point('1');
            wall_end = corners[i][j].get_point('2');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;

                        looking_at = '0';
                        cell_column = -1;
                        cell_row = -1;
                        intersection = temp_intersection;
                    }
                }
            }
            wall_start = corners[i][j].get_point('2');
            wall_end = corners[i][j].get_point('3');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;

                        looking_at = '0';
                        cell_column = -1;
                        cell_row = -1;
                        intersection = temp_intersection;
                    }
                }
            }
            wall_start = corners[i][j].get_point('3');
            wall_end = corners[i][j].get_point('4');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;

                        looking_at = '0';
                        cell_column = -1;
                        cell_row = -1;
                        intersection = temp_intersection;
                    }
                }
            }
            wall_start = corners[i][j].get_point('4');
            wall_end = corners[i][j].get_point('1');
            temp_intersection = findIntersection(sens_pos, end_sensor, wall_start, wall_end, temp_dist_measure, intersection_found);
            if (intersection_found){
                if (abs(dist_measure-temp_dist_measure) < tolerance_dist){
                    if (shortest_dist_measure > temp_dist_measure) {
                        shortest_dist_measure = temp_dist_measure;

                        looking_at = '0';
                        cell_column = -1;
                        cell_row = -1;
                        intersection = temp_intersection;
                    }
                }
            }
        }
    }

    // cout << "intersection: " << intersection[0] << ", " << intersection[1] << endl;

    if (looking_at != '0') {
        labyrinth[cell_column][cell_row].update_wall(looking_at, true);
        if (looking_at == 'N') {
            if ((cell_column-1) >= 0) {
                labyrinth[cell_column-1][cell_row].update_wall('S', true);
            }
        }
        if (looking_at == 'E') {
            if ((cell_row+1) < LABYRINTH_WIDTH) {
                labyrinth[cell_column][cell_row+1].update_wall('W', true);
            }
        }
        if (looking_at == 'S') {
            if ((cell_column+1) < LABYRINTH_HEIGHT) {
                labyrinth[cell_column+1][cell_row].update_wall('N', true);
            }
        }
        if (looking_at == 'W') {
            if ((cell_row-1) >= 0) {
                labyrinth[cell_column][cell_row-1].update_wall('E', true);
            }
        }
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 128);
        SDL_RenderDrawLine(renderer, p1, p2, p3, p4);
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

void RobotEst::get_offset_left(float& height, float& angle_rel, float alpha=M_PI/4, float beta=M_PI/2){
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
    height = (2*A)/L;

    angle_rel = acos(max(min(height/dist_e1, 1.0f), -1.0f));
    if (dist_e2*cos(alpha) < dist_e1){
        angle_rel *= -1;
    } else {}

  }

void RobotEst::get_offset_right(float& height, float& angle_rel, float alpha=M_PI/4, float beta=-M_PI/2){
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
    height = (2*A)/L;

    // gamma = acos(max(min(h/dist_e5, 1.0f), -1.0f));
    // if (dist_e4*cos(alpha) < dist_e5){
    //     angle_rel = 2*M_PI + beta + gamma;
    // } else {
    //     angle_rel = 2*M_PI + beta - gamma;
    // }

    angle_rel = acos(max(min(height/dist_e5, 1.0f), -1.0f));
    if (dist_e4*cos(alpha) < dist_e5){}
    else {
        angle_rel *= -1;
    }
}

void RobotEst::get_offset_front_left(float& height, float& angle_rel, float alpha=M_PI/4){
    float dist_e2 = sensL2.dist_measure + sensL2.offset_position;
    float dist_e3 = sensS.dist_measure + sensS.offset_position;
    float L = sqrt( pow(dist_e3 * cos(alpha) - dist_e2, 2) + pow(dist_e3*sin(alpha), 2) );
    float s = (dist_e2 + dist_e3 + L)/2;
    float A = sqrt( s*(s-dist_e2)*(s-dist_e3)*(s-L) );
    height = (2*A)/L;

    angle_rel = acos(max(min(height/dist_e3, 1.0f), -1.0f));
    if (dist_e2*cos(alpha) < dist_e3){}
    else {
        angle_rel *= -1;
    }
}


void RobotEst::get_offset_front_right(float& height, float& angle_rel, float alpha=M_PI/4){
    float dist_e3 = sensS.dist_measure + sensS.offset_position;
    float dist_e4 = sensR2.dist_measure + sensR2.offset_position;
    float L = sqrt( pow(dist_e3 * cos(alpha) - dist_e4, 2) + pow(dist_e3*sin(alpha), 2) );
    float s = (dist_e4 + dist_e3 + L)/2;
    float A = sqrt( s*(s-dist_e4)*(s-dist_e3)*(s-L) );
    height = (2*A)/L;

    angle_rel = acos(max(min(height/dist_e3, 1.0f), -1.0f));
    // cout << "L: " << L << "  s: " << s << "  A: " << A << "  dist_e4: " << dist_e4 << "  dist_e3: " << dist_e3 << "  height: " << height << "   angle_rel: " << angle_rel << endl;
    if (dist_e4*cos(alpha) < dist_e3){
        angle_rel *= -1;
    } else {}
}


void RobotEst::localization(float error_band=M_PI/6, float smoothing=0.1){
    char driving_dir;
    float height;
    float angle_rel;
    float num_updates = 0;

    vector <float> temp_rob_pos = {0, 0};
    float temp_rob_dir = 0; 

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
    if ((sensL.looking_at == sensL2.looking_at) && sensL.looking_at != '0') {
        get_offset_left(height, angle_rel, M_PI/4, M_PI/2);
        // # updating
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
        }
        if (driving_dir == 'N' && sensL.looking_at == 'W'){
            num_updates += 1;
            temp_rob_pos[0] += sensL.intersection[0] + height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving North" << endl;
        }

        else if (driving_dir == 'S' && sensL.looking_at == 'E'){
            num_updates += 1;
            temp_rob_pos[0] += sensL.intersection[0] - height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving South" << endl;
        }

        else if (driving_dir == 'W' && sensL.looking_at == 'S'){
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensL.intersection[1] - height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving West" << endl; 
        }
        else if (driving_dir == 'E' && sensL.looking_at == 'N') {
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensL.intersection[1] + height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving East" << endl;
        }
    }

    // 2 & 3
    if ((sensL2.looking_at == sensS.looking_at) && sensL2.looking_at != '0') {
        get_offset_front_left(height, angle_rel, M_PI/4);
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
        }
        if (driving_dir == 'N' && sensS.looking_at=='N'){
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensS.intersection[1] + height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'S' && sensS.looking_at=='S'){
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensS.intersection[1] - height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'W' && sensS.looking_at=='W'){
            num_updates += 1;
            temp_rob_pos[0] += sensS.intersection[0] + height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'E' && sensS.looking_at=='E') {
            num_updates += 1;
            temp_rob_pos[0] += sensS.intersection[0] - height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
        }
    }

    // 3 & 4
    if (sensS.looking_at == sensR2.looking_at && sensS.looking_at != '0') {
        get_offset_front_right(height, angle_rel, M_PI/4);
        // # updating
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
        }
        if (driving_dir == 'N' && sensS.looking_at=='N'){
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensS.intersection[1] + height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'S' && sensS.looking_at=='S'){
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensS.intersection[1] - height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'W' && sensS.looking_at=='W'){
            num_updates += 1;
            temp_rob_pos[0] += sensS.intersection[0] + height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'E' && sensS.looking_at=='E') {
            num_updates += 1;
            temp_rob_pos[0] += sensS.intersection[0] - height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
        }
    }

    // 4 & 5
    if ((sensR2.looking_at == sensR.looking_at) && sensR2.looking_at != '0'){
        get_offset_right(height, angle_rel, M_PI/4, -M_PI/2);
        // # updating
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
            return;
        }
        if (driving_dir == 'N' && sensR.looking_at=='E'){
            num_updates += 1;
            temp_rob_pos[0] += sensR2.intersection[0] - height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving North" << endl;

        } else if (driving_dir == 'S' && sensR.looking_at=='W') {
            num_updates += 1;
            temp_rob_pos[0] += sensR2.intersection[0] + height;
            temp_rob_pos[1] += rob_pos[1];
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving South" << endl;

        } else if (driving_dir == 'W' && sensR.looking_at=='S') {
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensR2.intersection[1] + height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving West" << endl;

        } else if (driving_dir == 'E' && sensR.looking_at=='N') {
            num_updates += 1;
            temp_rob_pos[0] += rob_pos[0];
            temp_rob_pos[1] += sensR2.intersection[1] - height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving East" << endl;
        }  
    }

    if (num_updates >= 1){
        // cout << "before: " << rob_pos[0] << ", " << rob_pos[1] << endl;
        rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(temp_rob_pos[0]/num_updates);
        rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(temp_rob_pos[1]/num_updates);
        // cout << "after: " << rob_pos[0] << ", " << rob_pos[1] << endl;
        rob_dir = (1-smoothing)*rob_dir + smoothing*(round(rob_dir/M_PI_2)*M_PI_2 + ((temp_rob_dir/num_updates)-M_PI));

        rob_dir = fmod(rob_dir, 2*M_PI);
        if (rob_dir < 0) {
            rob_dir += 2*M_PI;
        }
    }

    // update distance to front wall if one is seen.
    float distance; //TODO estimation is beeing pushed back due to imagining walls at max sensor range.
    if (sensS.looking_at != '0'){
        if (sensS.looking_at == 'N') {
            distance = (sensS.dist_measure + sensS.offset_position) * cos(sensS.sens_dir - 3*M_PI_2);
            rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(sensS.intersection[1] + distance);
        } else if (sensS.looking_at == 'E') {
            distance = (sensS.dist_measure + sensS.offset_position) * cos(sensS.sens_dir);
            rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(sensS.intersection[0] - distance);
        } else if (sensS.looking_at == 'S') {
            distance = (sensS.dist_measure + sensS.offset_position) * cos(sensS.sens_dir - M_PI_2);
            rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(sensS.intersection[1] - distance);
        } else if (sensS.looking_at == 'W') {
            distance = (sensS.dist_measure + sensS.offset_position) * cos(sensS.sens_dir - M_PI);
            rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(sensS.intersection[0] + distance);
        }
    }

    if (sensL2.looking_at != '0'){
        if (sensL2.looking_at == 'N') {
            distance = (sensL2.dist_measure + sensL2.offset_position) * cos(3*M_PI_2 - sensL2.sens_dir);
            rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(sensL2.intersection[1] + distance);
        } else if (sensL2.looking_at == 'E') {
            distance = (sensL2.dist_measure + sensL2.offset_position) * cos(sensL2.sens_dir);
            rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(sensL2.intersection[0] - distance);
        } else if (sensL2.looking_at == 'S') {
            distance = (sensL2.dist_measure + sensL2.offset_position) * cos(M_PI_2 - sensL2.sens_dir);
            rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(sensL2.intersection[1] - distance);
        } else if (sensL2.looking_at == 'W') {
            distance = (sensL2.dist_measure + sensL2.offset_position) * cos(M_PI - sensL2.sens_dir);
            rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(sensL2.intersection[0] + distance);
        }
    }

    if (sensR2.looking_at != '0'){
        if (sensR2.looking_at == 'N') {
            distance = (sensR2.dist_measure + sensR2.offset_position) * cos(3*M_PI_2 - sensR2.sens_dir);
            rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(sensR2.intersection[1] + distance);
        } else if (sensR2.looking_at == 'E') {
            distance = (sensR2.dist_measure + sensR2.offset_position) * cos(sensR2.sens_dir);
            rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(sensR2.intersection[0] - distance);
        } else if (sensR2.looking_at == 'S') {
            distance = (sensR2.dist_measure + sensR2.offset_position) * cos(M_PI_2 - sensR2.sens_dir);
            rob_pos[1] = (1-smoothing)*rob_pos[1] + smoothing*(sensR2.intersection[1] - distance);
        } else if (sensR2.looking_at == 'W') {
            distance = (sensR2.dist_measure + sensR2.offset_position) * cos(M_PI - sensR2.sens_dir);
            rob_pos[0] = (1-smoothing)*rob_pos[0] + smoothing*(sensR2.intersection[0] + distance);
        }
    }
}

void RobotEst::get_wheelspeeds(Robot mouse){
    speedLeft = mouse.speedLeft + gaussianNoise(0, ENCODER_VAR);
    speedRight = mouse.speedRight + gaussianNoise(0, ENCODER_VAR);
}


void RobotEst::updatePosition(SDL_Renderer *renderer, Robot mouse, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    float deltaTime = 1;
    float wheelRadius = 1;
    float velocity;
    float angularVelocity;
    
    // read out speed and sensor distances
    this->get_wheelspeeds(mouse);
    sensR.getDistanceToWall(mouse.sensR);
    sensR2.getDistanceToWall(mouse.sensR2);
    sensS.getDistanceToWall(mouse.sensS);
    sensL2.getDistanceToWall(mouse.sensL2);
    sensL.getDistanceToWall(mouse.sensL);
    
    // compare sensor distances to estimated distances
    sensR.compareDistanceToWall(renderer, rob_pos, labyrinth, corners);
    sensR2.compareDistanceToWall(renderer, rob_pos, labyrinth, corners);
    sensS.compareDistanceToWall(renderer, rob_pos, labyrinth, corners);
    sensL2.compareDistanceToWall(renderer, rob_pos, labyrinth, corners);
    sensL.compareDistanceToWall(renderer, rob_pos, labyrinth, corners);

    // Calculate the new position of the robot
    velocity = wheelRadius * (speedLeft + speedRight) / 2;
    angularVelocity = wheelRadius * (speedLeft - speedRight) / distance_wheels;

    // cout << "before: " << rob_dir;
    rob_dir = fmod(rob_dir + angularVelocity * deltaTime, 2*M_PI);
    if (rob_dir < 0) {
        rob_dir += 2*M_PI;
    }
    // cout << "   after: " << rob_dir << endl;
    rob_pos[0] += velocity * cos(rob_dir) * deltaTime;
    rob_pos[1] += velocity * sin(rob_dir) * deltaTime;

    // Calculate the new position of the sensors
    sensR.updatePosition(rob_pos, rob_dir);
    sensR2.updatePosition(rob_pos, rob_dir);
    sensS.updatePosition(rob_pos, rob_dir);
    sensL2.updatePosition(rob_pos, rob_dir);
    sensL.updatePosition(rob_pos, rob_dir);
    // cout << sensR.looking_at << " " << sensR.intersection[0] << "," << sensR.intersection[1] << endl;
}