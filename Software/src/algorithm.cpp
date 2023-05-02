#include <cmath>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <list>
#include <queue>
#include <stack>
#include <set>
#include <map>
#include "defs.h"
#include "utils.h"
#include "estimations.h"
#include "algorithm.h"

using namespace std;

float KP = 5;
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
    // cout << "error: " << error << endl;
    // cout << "output: " << output << endl;
    return output;
}

void PIDController::reset(){
    lastError = 0;
    totalError = 0;
}

bool findGoal(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> goal_cell) {
    for (int x = 1; x < LABYRINTH_WIDTH; x++) {
        for (int y = 1; y < LABYRINTH_HEIGHT; y++) {
            
            if( labyrinth[y][x].N < THRESHOLD_NO_WALL && 
                labyrinth[y][x].W < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].S < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].E < THRESHOLD_NO_WALL) {

                labyrinth[y][x].is_goal = true;
                labyrinth[y-1][x].is_goal = true;
                labyrinth[y][x-1].is_goal = true;
                labyrinth[y-1][x-1].is_goal = true;
                goal_cell = {x, y};
                
                cout << x << ", " << y << endl;
                cout << x-1 << ", " << y << endl;
                cout << x-1 << ", " << y-1 << endl;
                cout << x << ", " << y-1 << endl;

                // cout << !labyrinth[y][x].has_wall('N') << endl;
                // cout << !labyrinth[y][x].has_wall('W') << endl;
                // cout << !labyrinth[y][x-1].has_wall('S') << endl;
                // cout << !labyrinth[y][x-1].has_wall('E') << endl;
                cout << "_----------------_" << endl;

                return true;
            }
            
        }
    }
    return false;
}



stack<vector<int>> getPath(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start, vector<int> goal) {
    // if no goal is set just returns the path to the nearest unexplored cell
    vector<int> no_goal = {-1, -1};
    int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
    // Define a queue to store the cells to be checked
    queue<vector<int>> q;
    queue<vector<int>> q_next;
    q.push(start);
    int x;
    int y;
    // Define a map to store the parent of each cell
    map<vector<int>, vector<int>> parent;
    // Loop until the goal is reached or the queue is empty
    while (!q.empty()) {
        // Get the next cell from the queue
        vector<int> current = q.front();
        q.pop();
        x = current[0];
        y = current[1];
        // Check if the current cell is the goal
        if (current == goal || (goal == no_goal && !labyrinth[y][x].seen)) {
            stack<vector<int>> next_points;
            // Construct the path from the goal to the start
            next_points.push(current);
            while (current != start) {
                current = parent[current];
                next_points.push(current);
                // next_points.insert(next_points.begin(), current);
            }
            next_points.pop();
            printPath(next_points);
            return next_points;
        }
        // Check the north cell
        if ((y-1) >= 0 && grid[y-1][x] == 0) {
            if (x!=start[0] || (y-1)!=start[1]){
                if (labyrinth[y][x].N < 0.2) {
                    q_next.push({x, y-1});
                    parent[{x, y-1}] = current;
                }
            }
        }
        // Check the east cell
        if (x < LABYRINTH_WIDTH-1 && grid[y][x+1] == 0) {
            if ((x+1)!=start[0] || y!=start[1]){
                if (labyrinth[y][x].E < 0.2) {
                    q_next.push({x+1, y});
                    parent[{x+1, y}] = current;
                }
            }
        }
        // Check the south cell
        if (y < LABYRINTH_HEIGHT-1 && grid[y+1][x] == 0) {
            if (x!=start[0] || (y+1)!=start[1]){
                if (labyrinth[y][x].S < 0.2) {
                    q_next.push({x, y+1});
                    parent[{x, y+1}] = current;
                }
            }
        }
        // Check the west cell
        if ((x-1) >= 0 && grid[y][x-1] == 0) {
            if ((x-1)!=start[0] || y!=start[1]){
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
    return stack<vector<int>>();
} 

void printPath(queue<vector<int>> path) {
    // Loop until the path is empty
    while (!path.empty()) {
        // Get the next cell from the path
        vector<int> current = path.front();
        path.pop();
        // Print the coordinates of the cell
        cout << "(" << current[0] << ", " << current[1] << ")" << endl;
    }
    cout << "________________" << endl;
}

void printPath(stack<vector<int>> path) {
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


Planner::Planner() {
    start_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    current_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    goal_cell = {-1, -1};
    goal_found = false;
    point_reached = false;
    next_point = {-1, -1};
};

vector<float> Planner::drive_to(vector<int> next, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], PIDController PID){
    float turn_rate;
    float target_tolerance = 25;
    float dir_tolerance = M_PI/60; //3 degrees to either side

    vector<float> target = {-1.0, -1.0};
    target[0] = float(next[0]) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;
    target[1] = float(next[1]) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;

    // cout << "target x: " << target[0] << endl;
    // cout << "target y: " << target[1] << endl;
    // cout << "mouse x: " << mouseEst.rob_pos[0] << endl;
    // cout << "mouse y: " << mouseEst.rob_pos[1] << endl;

    // cout << "target x: " << target[0]-mouseEst.rob_pos[0] << endl;
    // cout << "target y: " << target[1]-mouseEst.rob_pos[1] << endl;

    float dir_to_target = atan2((target[1]-mouseEst.rob_pos[1]),(target[0]-mouseEst.rob_pos[0]));
    // cout << "before: " << dir_to_target << endl;
    if (dir_to_target > 0) {
        dir_to_target += 2* M_PI;
    } else if (dir_to_target <= 0) {
        dir_to_target *= 1;
    }
    // cout << "dir_to_target: " << dir_to_target << endl;
    dir_to_target = fmod(dir_to_target, 2*M_PI);
    if (dir_to_target < 0) {
        dir_to_target += 2*M_PI;
    }
    // cout << "after: " << dir_to_target << endl;

    if (distBetweenPoints(mouseEst.rob_pos, target) < target_tolerance) {
        labyrinthEst[next[1]][next[0]].seen = true;
        point_reached = true;
        return vector<float>{0.0, 0.0};
    } else if (abs(mouseEst.rob_dir - dir_to_target) < dir_tolerance) {
        // drive straight
        return vector<float>{this->turnPID.maxOutput, this->turnPID.maxOutput};
    } else {
        float dot_product = cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target);
        // turn_rate = PID.calculate(0, mouseEst.rob_dir - dir_to_target, 1);
        turn_rate = PID.calculate(1, dot_product, 1);
        if (turn_rate < 0){
            turn_rate *= -1;
        }
        float speed_forward = (this->turnPID.maxOutput-turn_rate)/2;
        // cout << "turn " << endl;
        // cout << "dir_to_target: " << dir_to_target << endl;
        if (((mouseEst.rob_dir - dir_to_target) > 0) || ((mouseEst.rob_dir - dir_to_target) < -M_PI)) {
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
    if (!goal_found) {
        goal_found = findGoal(labyrinthEst, goal_cell);
    }
    if (next_points.empty()) {
        if (goal_found) {
            if (current_cell==start_cell) {
                next_points = getPath(labyrinthEst, start_cell, goal_cell);
            } else {
                next_points = getPath(labyrinthEst, current_cell, start_cell);
            }
        } else {
            next_points = getPath(labyrinthEst, current_cell, vector<int>{-1, -1});
        }
    } else {
        if (point_reached){
            next_point = next_points.top();
            next_points.pop();
            point_reached = false;
            turnPID.reset();
        }
        // cout << next_point[0] << ", " << next_point[1] << endl;
        return drive_to(next_point, mouseEst, labyrinthEst, turnPID);
    }
    // cout << current_cell[0] << ", " << current_cell[1] << endl;
    return drive_to(current_cell, mouseEst, labyrinthEst, turnPID);
    
}
  


