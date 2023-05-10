#include <cmath>
#include <vector>
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

PIDController::PIDController() : kp(KP), ki(KI), kd(KD), maxOutput(PID_MAX), minOutput(PID_MIN), lastError(0), totalError(0) {}

PIDController::PIDController(float kp, float ki, float kd, float maxOutput, float minOutput) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
    lastError = 0;
    totalError = 0;
}

float PIDController::calculate(float setPoint, float processVariable, float deltaTime) {
    float error = setPoint - processVariable;
    float derivative = (error - lastError) / deltaTime;
    totalError += error * deltaTime;
    float output = kp * error + ki * totalError + kd * derivative;
    output = min(output, maxOutput);
    output = max(output, minOutput);
    lastError = error;
    return output;
}

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

                cout << "-------goal-------" << endl;
                cout << "(" << x-1 << ", " << y-1 << ")   " << "(" << x-1 << ", " << y << ")" << endl;
                cout << "(" << x << ", " << y-1 << ")   " << "(" << x << ", " << y << ")" << endl;
                cout << "------------------" << endl;
            }
            
        }
    }
}

vector<int> Planner::getPath(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> begin, char go_to) {
    // Finds path from begin to start (S), Goal (G) or Unexplored (U) and saves it in next_points. Also returns the cell the algorithm drives to.
    int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
    // Define a queue to store the cells to be checked
    queue<vector<int>> q;
    queue<vector<int>> q_next;
    q.push(begin);
    int x;
    int y;
    bool done = false;
    // Define a map to store the parent of each cell
    map<vector<int>, vector<int>> parent;
    // Loop until the end is reached or the queue is empty
    while (!q.empty()) {
        // Get the next cell from the queue
        vector<int> current = q.front();
        q.pop();
        x = current[0];
        y = current[1];

        switch (go_to)
        {
        case 'S':
            if (current == start_cell){
                done = true;
            }
            break;
        case 'G':
            if (current == goal_1 || current == goal_2 || current == goal_3 || current == goal_4){
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
            vector<int> driving_to = current;
            // Construct the path from the end to the begin
            if (current != begin) {
                vector<int> waypoint = current;
                next_points.push(current);
                current = parent[current];
                while (current != begin) {
                    if ((waypoint[0] == current[0] && current[0] == parent[current][0]) || (waypoint[1] == current[1] && current[1] == parent[current][1])) {
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
            if (x!=begin[0] || (y-1)!=begin[1]){
                if (labyrinth[y][x].N < 0.2) {
                    q_next.push({x, y-1});
                    parent[{x, y-1}] = current;
                }
            }
        }
        // Check the east cell
        if (x < LABYRINTH_WIDTH-1 && grid[y][x+1] == 0) {
            if ((x+1)!=begin[0] || y!=begin[1]){
                if (labyrinth[y][x].E < 0.2) {
                    q_next.push({x+1, y});
                    parent[{x+1, y}] = current;
                }
            }
        }
        // Check the south cell
        if (y < LABYRINTH_HEIGHT-1 && grid[y+1][x] == 0) {
            if (x!=begin[0] || (y+1)!=begin[1]){
                if (labyrinth[y][x].S < 0.2) {
                    q_next.push({x, y+1});
                    parent[{x, y+1}] = current;
                }
            }
        }
        // Check the west cell
        if ((x-1) >= 0 && grid[y][x-1] == 0) {
            if ((x-1)!=begin[0] || y!=begin[1]){
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
    return vector<int> {-1, -1};
} 

float Planner::calcPathLength(stack<vector<int>> path, vector<int> start) {
    int length = 0;
    vector<int> previous = start;
    vector<int> current;
    while (!path.empty()) {
        current = path.top();
        length += abs(previous[0] - current[0]) + abs(previous[1] - current[1]);
        previous = current;
        path.pop();
    }
    length += previous[0] - current[0] + previous[1] - current[1];
    return length;
}

void Planner::printPath(stack<vector<int>> path) {
    // Loop until the path is empty
    while (!path.empty()) {
        // Get the next cell from the path
        vector<int> current = path.top();
        path.pop();
        // Print the coordinates of the cell
        cout << "(" << current[0] << ", " << current[1] << ")" << endl;
    }
    cout << "________________" << endl;
}


vector<float> Planner::drive_to(vector<int> next, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], PIDController PID){
    float turn_rate;
    float target_tolerance = 25;
    float dir_tolerance = M_PI/60; //3 degrees to either side

    vector<float> target = {-1.0, -1.0};
    target[0] = float(next[0]) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;
    target[1] = float(next[1]) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;

    float dir_to_target = atan2((target[1]-mouseEst.rob_pos[1]),(target[0]-mouseEst.rob_pos[0]));
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
        labyrinthEst[next[1]][next[0]].is_seen = true;
        point_reached = true;
        return vector<float>{0.0, 0.0};
    } else if (abs(mouseEst.rob_dir - dir_to_target) < dir_tolerance) {
        // drive straight
        return vector<float>{this->turnPID.maxOutput, this->turnPID.maxOutput};
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
            return vector<float>{speed_forward-turn_rate, speed_forward+turn_rate};
        } else {
            // turn right
            return vector<float>{speed_forward+turn_rate, speed_forward-turn_rate};
        }
    }
}


vector<float> Planner::update(RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    current_cell[0] = int(round(((mouseEst.rob_pos[0] - CELL_SIZE/2 - WALL_WIDTH)/(CELL_SIZE + WALL_WIDTH))));
    current_cell[1] = int(round(((mouseEst.rob_pos[1] - CELL_SIZE/2 - WALL_WIDTH)/(CELL_SIZE + WALL_WIDTH))));
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
                stack<vector<int>> goal_path;
                getPath(labyrinthEst, start_cell, 'G');
                shortest_path_length = calcPathLength(next_points, start_cell);
                swap(next_points, goal_path);

                vector<int> next_unexplored;
                next_unexplored = getPath(labyrinthEst, current_cell, 'U');
                float unexplored_path_length = calcPathLength(next_points, current_cell);
                int manhatten_to_start = abs(next_unexplored[0]-start_cell[0])+abs(next_unexplored[1]-start_cell[1]);

                if(shortest_path_length <= (unexplored_path_length + manhatten_to_start + 1)){
                    fastest_path_found = true;
                    stack<vector<int>> empty;
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
    return vector<float>{0, 0};
}
  


