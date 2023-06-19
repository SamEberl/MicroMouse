#include <cmath>
#include <stdio.h>
#include <iostream>
#include <queue>
#include <stack>
#include <map>
#include "defs.h"
#include "utils.h"
#include "estimations.h"
#include "algorithm.h"

using namespace std;

float KP = 2;
float KI = 1;
float KD = 1;
float PID_MAX = 1.0;
float PID_MIN = -1.0;
float THRESHOLD_NO_WALL = 0.2;

float PID_calculate(PIDController pid, float setPoint, float processVariable, float deltaTime) {
    float error = setPoint - processVariable;
    float derivative = (error - pid.lastError) / deltaTime;
    pid.totalError += error * deltaTime;
    float output = pid.kp * error + pid.ki * pid.totalError + pid.kd * derivative;
    output = min(output, pid.maxOutput);
    output = max(output, pid.minOutput);
    pid.lastError = error;
    return output;
}

// CONTINUE HERE BY FINISHING UP THE PID

Planner::Planner(int start_x, int start_y) {
    start_cell = {start_x, start_y};
    current_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    goal_found = false;
    point_reached = false;
    shortest_path_length = 99999;
};

void Planner::findGoal(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    for (int x = 1; x < LABYRINTH_WIDTH; x++) {
        for (int y = 1; y < LABYRINTH_HEIGHT; y++) {
            
            if( labyrinth[y][x].N < THRESHOLD_NO_WALL && 
                labyrinth[y][x].W < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].S < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].E < THRESHOLD_NO_WALL) {

                goal_found = true;

                goal_1 = {x-1, y-1};
                goal_2 = {x, y-1};
                goal_3 = {x, y};
                goal_4 = {x-1, y};
            }
            
        }
    }
}

CellPos Planner::getPath(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CellPos begin, char go_to) {
    // Finds path from begin to start (S), Goal (G) or Unexplored (U) and saves it in next_points. Also returns the cell the algorithm drives to.
    int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
    // Define a queue to store the cells to be checked
    queue<CellPos> q;
    queue<CellPos> q_next;
    q.push(begin);
    int x;
    int y;
    bool done = false;
    // Define a map to store the parent of each cell
    map<CellPos, CellPos> parent;
    // Loop until the end is reached or the queue is empty
    while (!q.empty()) {
        // Get the next cell from the queue
        CellPos current = q.front();
        q.pop();
        x = current.x;
        y = current.y;

        switch (go_to)
        {
        case 'S':
            if ((current.x == start_cell.x) && (current.y == start_cell.y)){
                done = true;
            }
            break;
        case 'G':
            if (((current.x == goal_1.x) && (current.x == goal_1.x)) || 
                ((current.x == goal_2.x) && (current.x == goal_2.x)) || 
                ((current.x == goal_3.x) && (current.x == goal_3.x)) || 
                ((current.x == goal_4.x) && (current.x == goal_4.x))){
                done = true;
            }
            break;
        case 'U':
            if (!labyrinth[y][x].is_seen){
                done = true;
            }
            break;
        default:
            break;
        }

        // Check if the current cell is the end
        if (done) {
            CellPos driving_to = current;
            // Construct the path from the end to the begin
            if ((current.x != begin.x) || (current.y != begin.y)) {
                CellPos waypoint = current;
                next_points.push(current);
                current = parent[current];
                while ((current.x != begin.x) || (current.y != begin.y)) {
                    if ((waypoint.x == current.x && current.x == parent[current].x) || (waypoint.y == current.y && current.y == parent[current].y)) {
                        current = parent[current];
                    } else {
                        waypoint = current;
                        next_points.push(current);
                        current = parent[current];
                    }
                }
            } else {
                next_points.push(current);
            }
            point_reached = false;
            return driving_to;
        }
        // Check the north cell
        if ((y-1) >= 0 && grid[y-1][x] == 0) {
            if (x!=begin.x || (y-1)!=begin.y){
                if (labyrinth[y][x].N < 0.2) {
                    q_next.push({x, y-1});
                    parent[{x, y-1}] = current;
                }
            }
        }
        // Check the east cell
        if (x < LABYRINTH_WIDTH-1 && grid[y][x+1] == 0) {
            if ((x+1)!=begin.x || y!=begin.y){
                if (labyrinth[y][x].E < 0.2) {
                    q_next.push({x+1, y});
                    parent[{x+1, y}] = current;
                }
            }
        }
        // Check the south cell
        if (y < LABYRINTH_HEIGHT-1 && grid[y+1][x] == 0) {
            if (x!=begin.x || (y+1)!=begin.y){
                if (labyrinth[y][x].S < 0.2) {
                    q_next.push({x, y+1});
                    parent[{x, y+1}] = current;
                }
            }
        }
        // Check the west cell
        if ((x-1) >= 0 && grid[y][x-1] == 0) {
            if ((x-1)!=begin.x || y!=begin.y){
                if (labyrinth[y][x].W < 0.2) {
                    q_next.push({x-1, y});
                    parent[{x-1, y}] = current;
                }
            }
        }
        // Mark the current cell with the counter
        grid[y][x] = 1;
        // Get the next queue
        if (q.empty()) {
            swap(q, q_next);
        }
    }
    return CellPos {-1, -1};
} 

float Planner::calcPathLength(stack<CellPos> path, CellPos start) {
    int length = 0;
    CellPos previous = start;
    CellPos current;
    while (!path.empty()) {
        current = path.top();
        length += abs(previous.x - current.x) + abs(previous.y - current.y);
        previous = current;
        path.pop();
    }
    length += previous.x - current.x + previous.y - current.y;
    return length;
}

void Planner::printPath(stack<CellPos> path) {
    // Loop until the path is empty
    while (!path.empty()) {
        // Get the next cell from the path
        CellPos current = path.top();
        path.pop();
        // Print the coordinates of the cell
        cout << "(" << current.x << ", " << current.y << ")" << endl;
    }
    cout << "________________" << endl;
}


Point Planner::drive_to(CellPos next, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], PIDController PID){
    float turn_rate;
    float target_tolerance = 25;
    float dir_tolerance = M_PI/60; //3 degrees to either side

    Point target = {-1.0, -1.0};
    target.x = float(next.x) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;
    target.y = float(next.y) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;

    float dir_to_target = atan2((target.y-mouseEst.rob_pos.y),(target.x-mouseEst.rob_pos.x));
    if (dir_to_target > 0) {
        dir_to_target += 2* M_PI;
    } else if (dir_to_target <= 0) {
        dir_to_target *= 1;
    }
    dir_to_target = fmod(dir_to_target, 2*M_PI);
    if (dir_to_target < 0) {
        dir_to_target += 2*M_PI;
    }

    if (distBetweenPoints(mouseEst.rob_pos, target) < target_tolerance) {
        labyrinthEst[next.y][next.x].is_seen = true;
        point_reached = true;
        return Point{0.0, 0.0};
    } else if (abs(mouseEst.rob_dir - dir_to_target) < dir_tolerance) {
        // drive straight
        return Point{this->turnPID.maxOutput, this->turnPID.maxOutput};
    } else {
        // float dot_product = -2*acos((cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target)))/M_PI + 1;
        float dot_product = -acos((cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target)));

        turn_rate = PID.calculate(0, dot_product, 1);
        // if (turn_rate < 0){
        //     turn_rate *= -1;
        // }
        float speed_forward = (this->turnPID.maxOutput-turn_rate)/2;
        float temp_dir = fmod(mouseEst.rob_dir+M_PI, 2*M_PI);
        if (temp_dir < 0) {
            temp_dir += 2*M_PI;
        }
        float temp_vec_x = cos(mouseEst.rob_dir - dir_to_target);
        float temp_vec_y = sin(mouseEst.rob_dir - dir_to_target);
        if (atan2(temp_vec_y, temp_vec_x) > 0) {
            // turn left
            return Point{speed_forward-turn_rate, speed_forward+turn_rate};
        } else {
            // turn right
            return Point{speed_forward+turn_rate, speed_forward-turn_rate};
        }
    }
}


Point Planner::update(RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    CellPos current_cell = getCellFromPos(mouseEst.rob_pos);
    if (next_points.empty()) {
        if (!goal_found) {
            findGoal(labyrinthEst);
        }
        if (goal_found) {
            if (fastest_path_found) {
                if (current_cell==start_cell) {
                    getPath(labyrinthEst, start_cell, 'G');
                } else {
                    getPath(labyrinthEst, current_cell, 'S');
                }
            } else {
                stack<CellPos> goal_path;
                getPath(labyrinthEst, start_cell, 'G');
                shortest_path_length = calcPathLength(next_points, start_cell);
                swap(next_points, goal_path);

                CellPos next_unexplored;
                next_unexplored = getPath(labyrinthEst, current_cell, 'U');
                float unexplored_path_length = calcPathLength(next_points, current_cell);
                int manhatten_to_start = abs(next_unexplored.x-start_cell.x)+abs(next_unexplored.y-start_cell.y);

                if(shortest_path_length <= (unexplored_path_length + manhatten_to_start + 1)){
                    fastest_path_found = true;
                    stack<CellPos> empty;
                    swap(next_points, empty);
                } 
            }
        } else {
            getPath(labyrinthEst, current_cell, 'U');
        }
        if (!next_points.empty()) {
            printPath(next_points);
        }
    } else {
        if (point_reached){
            next_points.pop();
            point_reached = false;
        } else {
            return drive_to(next_points.top(), mouseEst, labyrinthEst, turnPID);
        }

    }
    return Point{0, 0};
}
  


