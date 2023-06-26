#include <stdio.h>
#include <iostream>
#include <math.h>
#include "estimations.h"
#include "defs.h"
#include "utils.h"

#define _USE_MATH_DEFINES

using namespace std;

float WALL_RETAIN = 0.99;
float POS_RETAIN = 0.999;
float SENS_VAR = 5.0;
float ENCODER_VAR = 0.05;
float THRESHOLD_WALL_SEEN = 0.6;

float MIN_CORNER_DIST = 20;
float TOLERANCE_DIST = 15;
float IGNORE_MAX = 20;

float L_ERROR_BAND= M_PI/6;
float L_SMOOTHING= 0.03; //smoothing for localization


void CornerEst_initialize(CornerEst& cornerEst, float row, float col) {
    cornerEst.p1.x = 0.0f;
    cornerEst.p1.y = 0.0f;
    cornerEst.p2.x = 0.0f;
    cornerEst.p2.y = 0.0f;
    cornerEst.p3.x = 0.0f;
    cornerEst.p3.y = 0.0f;
    cornerEst.p4.x = 0.0f;
    cornerEst.p4.y = 0.0f;

    cornerEst.p1.x = col * (CELL_SIZE + WALL_WIDTH);
    cornerEst.p1.y = row * (CELL_SIZE + WALL_WIDTH);
    cornerEst.p2.x = (col * (CELL_SIZE + WALL_WIDTH)) + WALL_WIDTH;
    cornerEst.p2.y = row * (CELL_SIZE + WALL_WIDTH);
    cornerEst.p3.x = (col * (CELL_SIZE + WALL_WIDTH)) + WALL_WIDTH;
    cornerEst.p3.y = (row * (CELL_SIZE + WALL_WIDTH)) + WALL_WIDTH;
    cornerEst.p4.x = col * (CELL_SIZE + WALL_WIDTH);
    cornerEst.p4.y = (row * (CELL_SIZE + WALL_WIDTH)) + WALL_WIDTH;
}

void CellEst_initialize(CellEst& cellEst, float row, float col) {
    float x_pos = col * (CELL_SIZE + WALL_WIDTH) + WALL_WIDTH;
    float y_pos = row * (CELL_SIZE + WALL_WIDTH) + WALL_WIDTH;

    cellEst.p1.x = x_pos;
    cellEst.p1.y = y_pos;
    cellEst.p2.x = x_pos + CELL_SIZE;
    cellEst.p2.y = y_pos;
    cellEst.p3.x = x_pos + CELL_SIZE;
    cellEst.p3.y = y_pos + CELL_SIZE;
    cellEst.p4.x = x_pos;
    cellEst.p4.y = y_pos + CELL_SIZE;

    if (row == 0) {
        cellEst.N = 1.0;
    }
    if (col == LABYRINTH_WIDTH - 1) {
        cellEst.E = 1.0;
    }
    if (row == LABYRINTH_HEIGHT - 1) {
        cellEst.S = 1.0;
    }
    if (col == 0) {
        cellEst.W = 1.0;
    }
}

void CellEst_update_wall(CellEst& cellEst, char direction, bool wallPresent) {
    switch (direction) {
        case 'N':
            cellEst.N = WALL_RETAIN * cellEst.N + (1 - WALL_RETAIN) * wallPresent;
            break;
        case 'E':
            cellEst.E = WALL_RETAIN * cellEst.E + (1 - WALL_RETAIN) * wallPresent;
            break;
        case 'S':
            cellEst.S = WALL_RETAIN * cellEst.S + (1 - WALL_RETAIN) * wallPresent;
            break;
        case 'W':
            cellEst.W = WALL_RETAIN * cellEst.W + (1 - WALL_RETAIN) * wallPresent;
            break;
    }
}

bool CellEst_has_wall(const CellEst& cellEst, char direction) {
    switch (direction) {
        case 'N':
            return cellEst.N > THRESHOLD_WALL_SEEN;
        case 'E':
            return cellEst.E > THRESHOLD_WALL_SEEN;
        case 'S':
            return cellEst.S > THRESHOLD_WALL_SEEN;
        case 'W':
            return cellEst.W > THRESHOLD_WALL_SEEN;
        default:
            return true;
    }
}

void SensorEst_init(SensorEst& sensorEst, Point& robot_pos, float robot_dir, float offset_direction_, float offset_position_) {
    sensorEst.dist_measure = SENSOR_RANGE;
    sensorEst.offset_direction = offset_direction_;
    sensorEst.sens_dir = robot_dir + sensorEst.offset_direction;
    sensorEst.offset_position = offset_position_;
    sensorEst.sens_pos.x = robot_pos.x + cos(sensorEst.sens_dir) * sensorEst.offset_position;
    sensorEst.sens_pos.y = robot_pos.y + sin(sensorEst.sens_dir) * sensorEst.offset_position;
}

void SensorEst_updatePosition(SensorEst& sensorEst, Point& robot_pos, float robot_dir) {
    sensorEst.sens_dir = robot_dir + sensorEst.offset_direction;
    sensorEst.sens_pos.x = robot_pos.x + cos(sensorEst.sens_dir) * sensorEst.offset_position;
    sensorEst.sens_pos.y = robot_pos.y + sin(sensorEst.sens_dir) * sensorEst.offset_position;
}

void compareDistanceToWall(SensorEst& sensor, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    // Function to see if an intersection was expected and if yes to update the belief about a wall being there.
    sensor.looking_at = '0';
    float shortest_dist_measure = SENSOR_RANGE;
    float temp_dist_measure = SENSOR_RANGE;
    bool intersection_found = false;
    Point cell_index;
    Point intersection;
    Point temp_intersection;
    Point end_sensor = {sensor.sens_pos.x + cos(sensor.sens_dir) * SENSOR_RANGE, sensor.sens_pos.y + sin(sensor.sens_dir) * SENSOR_RANGE};
    Point wall_start;
    Point wall_end;

    CellPos current_cell;
    current_cell = getCellFromPos(sensor.sens_pos);

    for (int i = 0; i < LABYRINTH_WIDTH; i += 1) {
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1) {
            wall_start = labyrinth[i][j].p1;
            wall_end = labyrinth[i][j].p2;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (sensor.dist_measure < (SENSOR_RANGE - IGNORE_MAX)) {
                        if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                            shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                            if ((distBetweenPoints(temp_intersection, wall_start) > MIN_CORNER_DIST) && (distBetweenPoints(temp_intersection, wall_end) > MIN_CORNER_DIST)) {
                                sensor.looking_at = 'N';
                                cell_index.x = j;
                                cell_index.y = i;
                                intersection = temp_intersection;
                            }
                        }
                    }
                } else if (sensor.dist_measure > (temp_dist_measure + TOLERANCE_DIST)) {
                    CellEst_update_wall(&labyrinth[i][j], 'N', false);
                    if (i - 1 >= 0) {
                        CellEst_update_wall(&labyrinth[i - 1][j], 'S', false);
                    }
                }
            }
            wall_start = labyrinth[i][j].p2;
            wall_end = labyrinth[i][j].p3;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (sensor.dist_measure < (SENSOR_RANGE - IGNORE_MAX)) {
                        if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                            shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                            if ((distBetweenPoints(temp_intersection, wall_start) > MIN_CORNER_DIST) && (distBetweenPoints(temp_intersection, wall_end) > MIN_CORNER_DIST)) {
                                sensor.looking_at = 'E';
                                cell_index.x = j;
                                cell_index.y = i;
                                intersection = temp_intersection;
                            }
                        }
                    }
                } else if (sensor.dist_measure > (temp_dist_measure + TOLERANCE_DIST)) {
                    CellEst_update_wall(&labyrinth[i][j], 'E', false);
                    if (j + 1 < LABYRINTH_WIDTH) {
                        CellEst_update_wall(&labyrinth[i][j + 1], 'W', false);
                    }
                }
            }
            wall_start = labyrinth[i][j].p3;
            wall_end = labyrinth[i][j].p4;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (sensor.dist_measure < (SENSOR_RANGE - IGNORE_MAX)) {
                        if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                            shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                            if ((distBetweenPoints(temp_intersection, wall_start) > MIN_CORNER_DIST) && (distBetweenPoints(temp_intersection, wall_end) > MIN_CORNER_DIST)) {
                                sensor.looking_at = 'S';
                                cell_index.x = j;
                                cell_index.y = i;
                                intersection = temp_intersection;
                            }
                        }
                    }
                } else if (sensor.dist_measure > (temp_dist_measure + TOLERANCE_DIST)) {
                    CellEst_update_wall(&labyrinth[i][j], 'S', false);
                    if (i + 1 < LABYRINTH_HEIGHT) {
                        CellEst_update_wall(&labyrinth[i + 1][j], 'N', false);
                    }
                }
            }
            wall_start = labyrinth[i][j].p4;
            wall_end = labyrinth[i][j].p1;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (sensor.dist_measure < (SENSOR_RANGE - IGNORE_MAX)) {
                        if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                            shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                            if ((distBetweenPoints(temp_intersection, wall_start) > MIN_CORNER_DIST) && (distBetweenPoints(temp_intersection, wall_end) > MIN_CORNER_DIST)) {
                                sensor.looking_at = 'W';
                                cell_index.x = j;
                                cell_index.y = i;
                                intersection = temp_intersection;
                            }
                        }
                    }
                } else if (sensor.dist_measure > (temp_dist_measure + TOLERANCE_DIST)) {
                    CellEst_update_wall(&labyrinth[i][j], 'W', false);
                    if (j - 1 >= 0) {
                        CellEst_update_wall(&labyrinth[i][j - 1], 'E', false);
                    }
                }
            }
        }
    }

    // Check intersection with corners
    for (int i = 0; i < LABYRINTH_WIDTH + 1; i += 1) {
        for (int j = 0; j < LABYRINTH_HEIGHT + 1; j += 1) {
            wall_start = corners[i][j].p1;
            wall_end = corners[i][j].p2;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                        shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                        sensor.looking_at = '0';
                        intersection = temp_intersection;
                    }
                }
            }
            wall_start = corners[i][j].p2;
            wall_end = corners[i][j].p3;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                        shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                        sensor.looking_at = '0';
                        intersection = temp_intersection;
                    }
                }
            }
            wall_start = corners[i][j].p3;
            wall_end = corners[i][j].p4;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                        shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                        sensor.looking_at = '0';
                        intersection = temp_intersection;
                    }
                }
            }
            wall_start = corners[i][j].p4;
            wall_end = corners[i][j].p1;
            findIntersection(sensor.sens_pos, end_sensor, wall_start, wall_end, &temp_dist_measure, &intersection_found, &temp_intersection);
            if (intersection_found) {
                if (fabs(sensor.dist_measure - temp_dist_measure) < TOLERANCE_DIST) {
                    if (shortest_dist_measure > fabs(sensor.dist_measure - temp_dist_measure)) {
                        shortest_dist_measure = fabs(sensor.dist_measure - temp_dist_measure);
                        sensor.looking_at = '0';
                        intersection = temp_intersection;
                    }
                }
            }
        }
    }

    // Switch wall if "wrong" side is assumed to be seen
    if (sensor.looking_at !='0') {
        if (sensor.looking_at == 'N') {
            if (cell_index.y > current_cell.y) {
                sensor.looking_at = 'S';
                cell_index.y -= 1;
            }
        }
        if (sensor.looking_at == 'E') {
            if ((cell_index.x < current_cell.x)) {
                sensor.looking_at = 'W';
                cell_index.x += 1;
            }
        }
        if (sensor.looking_at == 'S') {
            if (cell_index.y < current_cell.y) {
                sensor.looking_at = 'N';
                cell_index.y += 1;
            }
        }
        if (sensor.looking_at == 'W') {
            if (cell_index.x > current_cell.x) {
                sensor.looking_at = 'E';
                cell_index.x -= 1;
            }
        }
    }

    // update wall that is seen and it's opposite side
    if (sensor.looking_at != '0') {
        CellEst_update_wall(&labyrinth[int(cell_index.x)][int(cell_index.y)], sensor.looking_at, true);
        if (sensor.looking_at == 'N') {
            if ((cell_index.y-1) >= 0) {
                CellEst_update_wall(&labyrinth[int(cell_index.y-1)][int(cell_index.x)], 'S', true);
            }
        }
        if (sensor.looking_at == 'E') {
            if ((cell_index.x+1) < LABYRINTH_WIDTH) {
                CellEst_update_wall(&labyrinth[int(cell_index.y)][int(cell_index.x+1)], 'W', true);
            }
        }
        if (sensor.looking_at == 'S') {
            if ((cell_index.y+1) < LABYRINTH_HEIGHT) {
                CellEst_update_wall(&labyrinth[int(cell_index.y+1)][int(cell_index.x)], 'N', true);
            }
        }
        if (sensor.looking_at == 'W') {
            if ((cell_index.x-1) >= 0) {
                CellEst_update_wall(&labyrinth[int(cell_index.y)][int(cell_index.x-1)], 'E', true);
            }
        }
    }
}


void RobotEst_init(RobotEst& rob, float x, float y, float direction_, float distance_wheels_, float width_, float height_) {
    RobotEst* rob = (RobotEst*) malloc(sizeof(RobotEst));

    rob.rob_pos.x = x;
    rob.rob_pos.y = y;
    rob.rob_dir = direction_;
    rob.distance_wheels = distance_wheels_;
    rob.width = width_;
    rob.height = height_;

    SensorEst_init(&(rob.sensR), &(rob.rob_pos), rob.rob_dir, M_PI/2, rob.width/2);
    SensorEst_init(&(rob.sensR2), &(rob.rob_pos), rob.rob_dir, M_PI/4, rob.height/2);
    SensorEst_init(&(rob.sensS), &(rob.rob_pos), rob.rob_dir, 0.0, rob.height/2);
    SensorEst_init(&(rob.sensL2), &(rob.rob_pos), rob.rob_dir, -M_PI/4, rob.height/2);
    SensorEst_init(&(rob.sensL), &(rob.rob_pos), rob.rob_dir, -M_PI/2, rob.width/2);
}

void RobotEst_get_offset_and_angle(float* height, float* angle_rel, float dist_1, float dist_2, float alpha) {
    // implementation remains same
    float L = sqrt( pow(dist_2 * cos(alpha) - dist_1, 2) + pow(dist_2*sin(alpha), 2)) ;
    float s = (dist_1 + dist_2 + L)/2;
    float A = sqrt( s * (s-dist_1) * (s-dist_2) * (s-L) );
    *height = (2*A)/L;

    *angle_rel = acos(fmaxf(fminf(*height/dist_1, 1.0f), -1.0f));
    if (alpha > 0) {
        if (dist_2*cos(alpha) < dist_1) {
            *angle_rel *= -1;
        }
    } else {
        if (dist_2*cos(alpha) > dist_1) {
            *angle_rel *= -1;
        }
    }
}


void localization(RobotEst& rob){
    char driving_dir;
    float height;
    float angle_rel;
    float num_updates = 0;

    Point temp_rob_pos;
    temp_rob_pos.x = 0;
    temp_rob_pos.y = 0;
    float temp_rob_dir = 0; 

    // determine in which direction we are driving
    if (((2*M_PI - L_ERROR_BAND) < rob.rob_dir) || (rob.rob_dir < L_ERROR_BAND)) { //# yes, it is 'or'
        driving_dir = 'E';
    } else if ((3*M_PI/2 - L_ERROR_BAND < rob.rob_dir) && (rob.rob_dir < 3*M_PI/2 + L_ERROR_BAND)) {
        driving_dir = 'N';
    } else if ((M_PI - L_ERROR_BAND < rob.rob_dir) && (rob.rob_dir < M_PI + L_ERROR_BAND)) {
        driving_dir = 'W';
    } else if ((M_PI/2 - L_ERROR_BAND < rob.rob_dir) && (rob.rob_dir < M_PI/2 + L_ERROR_BAND)) {
        driving_dir = 'S';
    } else {
    // if we are currently turning, don't update
        return;
    }

    // TODO make updates depending on which wall is seen. h always updates y for N&S. h always updates x for W&E

    // 1 & 2
    if ((rob.sensL.looking_at == rob.sensL2.looking_at) && rob.sensL.looking_at != '0') {
        float dist_s1 = rob.sensL.dist_measure + rob.sensL.offset_position;
        float dist_s2 = rob.sensL2.dist_measure + rob.sensL2.offset_position;
        RobotEst_get_offset_and_angle(&height, &angle_rel, dist_s1, dist_s2, M_PI/4);
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
        }
        if (driving_dir == 'N' && rob.sensL.looking_at == 'W'){
            num_updates += 1;
            temp_rob_pos.x += rob.sensL.intersection.x + height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving North" << endl;
        }

        else if (driving_dir == 'S' && rob.sensL.looking_at == 'E'){
            num_updates += 1;
            temp_rob_pos.x += rob.sensL.intersection.x - height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving South" << endl;
        }

        else if (driving_dir == 'W' && rob.sensL.looking_at == 'S'){
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensL.intersection.y - height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving West" << endl; 
        }
        else if (driving_dir == 'E' && rob.sensL.looking_at == 'N') {
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensL.intersection.y + height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the left while moving East" << endl;
        }
    }

    // 2 & 3
    if ((rob.sensL2.looking_at == rob.sensS.looking_at) && rob.sensL2.looking_at != '0') {
        float dist_s2 = rob.sensL2.dist_measure + rob.sensL2.offset_position;
        float dist_s3 = rob.sensS.dist_measure + rob.sensS.offset_position;
        RobotEst_get_offset_and_angle(&height, &angle_rel, dist_s3, dist_s2, -M_PI/4);
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
        }
        if (driving_dir == 'N' && rob.sensS.looking_at=='N'){
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensS.intersection.y + height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'S' && rob.sensS.looking_at=='S'){
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensS.intersection.y - height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'W' && rob.sensS.looking_at=='W'){
            num_updates += 1;
            temp_rob_pos.x += rob.sensS.intersection.x + height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'E' && rob.sensS.looking_at=='E') {
            num_updates += 1;
            temp_rob_pos.x += rob.sensS.intersection.x - height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
        }
    }

    // 3 & 4
    if (rob.sensS.looking_at == rob.sensR2.looking_at && rob.sensS.looking_at != '0') {
        float dist_s3 = rob.sensS.dist_measure + rob.sensS.offset_position;
        float dist_s4 = rob.sensR2.dist_measure + rob.sensR2.offset_position;
        RobotEst_get_offset_and_angle(&height, &angle_rel, dist_s3, dist_s4, M_PI/4);
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
        }
        if (driving_dir == 'N' && rob.sensS.looking_at=='N'){
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensS.intersection.y + height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'S' && rob.sensS.looking_at=='S'){
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensS.intersection.y - height;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'W' && rob.sensS.looking_at=='W'){
            num_updates += 1;
            temp_rob_pos.x += rob.sensS.intersection.x + height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
        }

        else if (driving_dir == 'E' && rob.sensS.looking_at=='E') {
            num_updates += 1;
            temp_rob_pos.x += rob.sensS.intersection.x - height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
        }
    }

    // 4 & 5
    if ((rob.sensR2.looking_at == rob.sensR.looking_at) && rob.sensR2.looking_at != '0'){
        float dist_s4 = rob.sensR2.dist_measure + rob.sensR2.offset_position;
        float dist_s5 = rob.sensR.dist_measure + rob.sensR.offset_position;
        RobotEst_get_offset_and_angle(&height, &angle_rel, dist_s5, dist_s4, -M_PI/4);
        if((height != height) || (angle_rel != angle_rel)){
            cout << "NAN found!" << endl;
            return;
        }
        if (driving_dir == 'N' && rob.sensR.looking_at=='E'){
            num_updates += 1;
            temp_rob_pos.x += rob.sensR2.intersection.x - height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving North" << endl;

        } else if (driving_dir == 'S' && rob.sensR.looking_at=='W') {
            num_updates += 1;
            temp_rob_pos.x += rob.sensR2.intersection.x + height;
            temp_rob_pos.y += rob.rob_pos.y;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving South" << endl;

        } else if (driving_dir == 'W' && rob.sensR.looking_at=='S') {
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensR2.intersection.y + height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving West" << endl;

        } else if (driving_dir == 'E' && rob.sensR.looking_at=='N') {
            num_updates += 1;
            temp_rob_pos.x += rob.rob_pos.x;
            temp_rob_pos.y += rob.sensR2.intersection.y - height;
            temp_rob_dir += angle_rel + M_PI;
            // cout << "Wall to the right while moving East" << endl;
        }  
    }

    if (num_updates >= 1){
        rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(temp_rob_pos.x/num_updates);
        rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(temp_rob_pos.y/num_updates);
        rob.rob_dir = (1-L_SMOOTHING)*rob.rob_dir + L_SMOOTHING*(round(rob.rob_dir/M_PI_2)*M_PI_2 + ((temp_rob_dir/num_updates)-M_PI));

        rob.rob_dir = fmod(rob.rob_dir, 2*M_PI);
        if (rob.rob_dir < 0) {
            rob.rob_dir += 2*M_PI;
        }
    }

    // // update distance to front wall if one is seen.
    float distance;
    if (rob.sensS.looking_at != '0'){
        if (rob.sensS.looking_at == 'N') {
            distance = (rob.sensS.dist_measure + rob.sensS.offset_position) * cos(rob.sensS.sens_dir - 3*M_PI_2);
            rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(rob.sensS.intersection.y + distance);
        } else if (rob.sensS.looking_at == 'E') {
            distance = (rob.sensS.dist_measure + rob.sensS.offset_position) * cos(rob.sensS.sens_dir);
            rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(rob.sensS.intersection.x - distance);
        } else if (rob.sensS.looking_at == 'S') {
            distance = (rob.sensS.dist_measure + rob.sensS.offset_position) * cos(rob.sensS.sens_dir - M_PI_2);
            rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(rob.sensS.intersection.y - distance);
        } else if (rob.sensS.looking_at == 'W') {
            distance = (rob.sensS.dist_measure + rob.sensS.offset_position) * cos(rob.sensS.sens_dir - M_PI);
            rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(rob.sensS.intersection.x + distance);
        }
    }

    if (rob.sensL2.looking_at != '0'){
        if (rob.sensL2.looking_at == 'N') {
            distance = (rob.sensL2.dist_measure + rob.sensL2.offset_position) * cos(rob.sensL2.sens_dir - 3*M_PI_2);
            rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(rob.sensL2.intersection.y + distance);
        } else if (rob.sensL2.looking_at == 'E') {
            distance = (rob.sensL2.dist_measure + rob.sensL2.offset_position) * cos(rob.sensL2.sens_dir);
            rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(rob.sensL2.intersection.x - distance);
        } else if (rob.sensL2.looking_at == 'S') {
            distance = (rob.sensL2.dist_measure + rob.sensL2.offset_position) * cos(rob.sensL2.sens_dir - M_PI_2);
            rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(rob.sensL2.intersection.y - distance);
        } else if (rob.sensL2.looking_at == 'W') {
            distance = (rob.sensL2.dist_measure + rob.sensL2.offset_position) * cos(rob.sensL2.sens_dir - M_PI);
            rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(rob.sensL2.intersection.x + distance);
        }
    }

    if (rob.sensR2.looking_at != '0'){
        if (rob.sensR2.looking_at == 'N') {
            distance = (rob.sensR2.dist_measure + rob.sensR2.offset_position) * cos(rob.sensR2.sens_dir - 3*M_PI_2);
            rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(rob.sensR2.intersection.y + distance);
        } else if (rob.sensR2.looking_at == 'E') {
            distance = (rob.sensR2.dist_measure + rob.sensR2.offset_position) * cos(rob.sensR2.sens_dir);
            rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(rob.sensR2.intersection.x - distance); 
        } else if (rob.sensR2.looking_at == 'S') {
            distance = (rob.sensR2.dist_measure + rob.sensR2.offset_position) * cos(rob.sensR2.sens_dir - M_PI_2);
            rob.rob_pos.y = (1-L_SMOOTHING)*rob.rob_pos.y + L_SMOOTHING*(rob.sensR2.intersection.y - distance);
        } else if (rob.sensR2.looking_at == 'W') {
            distance = (rob.sensR2.dist_measure + rob.sensR2.offset_position) * cos(rob.sensR2.sens_dir - M_PI);
            rob.rob_pos.x = (1-L_SMOOTHING)*rob.rob_pos.x + L_SMOOTHING*(rob.sensR2.intersection.x + distance);
        }
    }
}


void RobotEst_updatePosition(RobotEst& rob, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    float deltaTime = 1;
    float wheelRadius = 1;
    float velocity;
    float angularVelocity;
    
    // read out speed and rob.sensor distances
    getMotorVelocity(0, rob.speedLeft);
    getMotorVelocity(1, rob.speedRight);

    getSensorVal(1, rob.sensR)
    getSensorVal(2, rob.sensR2)
    getSensorVal(3, rob.sensS)
    getSensorVal(4, rob.sensL2)
    getSensorVal(5, rob.sensL)

    // compare sensor distances to estimated distances
    compareDistanceToWall(rob.sensR, labyrinth, corners);
    compareDistanceToWall(rob.sensR2, labyrinth, corners);
    compareDistanceToWall(rob.sensS, labyrinth, corners);
    compareDistanceToWall(rob.sensL2, labyrinth, corners);
    compareDistanceToWall(rob.sensL, labyrinth, corners);

    // Calculate the new position of the robot
    velocity = wheelRadius * (rob.speedLeft + rob.speedRight) / 2;
    angularVelocity = wheelRadius * (rob.speedLeft - rob.speedRight) / DISTANCE_WHEELS;

    rob.rob_dir = fmod(rob.rob_dir + angularVelocity * deltaTime, 2*M_PI);
    if (rob.rob_dir < 0) {
        rob.rob_dir += 2*M_PI;
    }
    rob.rob_pos.x += velocity * cos(rob.rob_dir) * deltaTime;
    rob.rob_pos.y += velocity * sin(rob.rob_dir) * deltaTime;

    // Calculate the new position of the sensors
    SensorEst_updatePosition(rob.sensR, rob.rob_pos, rob.rob_dir);
    SensorEst_updatePosition(rob.sensR2, rob.rob_pos, rob.rob_dir);
    SensorEst_updatePosition(rob.sensS, rob.rob_pos, rob.rob_dir);
    SensorEst_updatePosition(rob.sensL2, rob.rob_pos, rob.rob_dir);
    SensorEst_updatePosition(rob.sensL, rob.rob_pos, rob.rob_dir);
}
