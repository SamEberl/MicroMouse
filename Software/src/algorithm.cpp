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
    // cout << "error: " << error << endl;
    // cout << "output: " << output << endl;
    return output;
}

void PIDController::reset(){
    lastError = 0;
    totalError = 0;
}

Planner::Planner() {
    start_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    current_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    goal_found = false;
    fastest_path_found;
    point_reached = false;
    shortest_path_length = 99999;
    next_point = {-1, -1};
};

void Planner::findGoal(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    for (int x = 1; x < LABYRINTH_WIDTH; x++) {
        for (int y = 1; y < LABYRINTH_HEIGHT; y++) {
            
            if( labyrinth[y][x].N < THRESHOLD_NO_WALL && 
                labyrinth[y][x].W < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].S < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].E < THRESHOLD_NO_WALL) {

                goal_found = true;

                labyrinth[y][x].is_goal = true;
                labyrinth[y-1][x].is_goal = true;
                labyrinth[y][x-1].is_goal = true;
                labyrinth[y-1][x-1].is_goal = true;

                cout << "-----goal-------" << endl;
                cout << x << ", " << y << endl;
                cout << x-1 << ", " << y << endl;
                cout << x-1 << ", " << y-1 << endl;
                cout << x << ", " << y-1 << endl;
                cout << "----------------" << endl;
            }
            
        }
    }
}

stack<vector<int>> Planner::getPath(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start, bool to_goal) {
    int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
    // Define a queue to store the cells to be checked
    queue<vector<int>> q;
    queue<vector<int>> q_next;
    q.push(start);
    int x;
    int y;
    bool end_found;
    // Define a map to store the parent of each cell
    map<vector<int>, vector<int>> parent;
    // Loop until the end is reached or the queue is empty
    while (!q.empty()) {
        // Get the next cell from the queue
        vector<int> current = q.front();
        q.pop();
        x = current[0];
        y = current[1];

        // Check if the current cell is the end
        if (((labyrinth[y][x].is_goal) && to_goal) || (!labyrinth[y][x].seen && !to_goal)) {
            stack<vector<int>> next_points;
            // Construct the path from the end to the start

            // next_points.push(current);
            if (current != start) {
                vector<int> waypoint = current;
                next_points.push(current);
                current = parent[current];
                while (current != start) {
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
            // next_points.pop();
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

stack<vector<int>> Planner::getPathPoint(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start, vector<int> end) {
    int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
    // Define a queue to store the cells to be checked
    queue<vector<int>> q;
    queue<vector<int>> q_next;
    q.push(start);
    int x;
    int y;
    bool end_found;
    // Define a map to store the parent of each cell
    map<vector<int>, vector<int>> parent;
    // Loop until the end is reached or the queue is empty
    while (!q.empty()) {
        // Get the next cell from the queue
        vector<int> current = q.front();
        q.pop();
        x = current[0];
        y = current[1];

        // Check if the current cell is the end
        if (current == end) {
            stack<vector<int>> next_points;
            // Construct the path from the end to the start

            // next_points.push(current);
            if (current != start) {
                vector<int> waypoint = current;
                next_points.push(current);
                current = parent[current];
                while (current != start) {
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
            // next_points.pop();
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

float Planner::calcPathLength(stack<vector<int>> path, vector<int> start) {
    int length = 0;
    vector<int> previous = start;
    vector<int> current = path.top();
     while (!path.empty()) {
        length += previous[0] - current[0] + previous[1] - current[1];
        // Get the next cell from the path
        // vector<int> current = path.front();
        previous = current;
        vector<int> current = path.top();
        path.pop();
    }
    length += previous[0] - current[0] + previous[1] - current[1];
    return length;
}

void Planner::printPath(stack<vector<int>> path) {
    // Loop until the path is empty
    while (!path.empty()) {
        // Get the next cell from the path
        // vector<int> current = path.front();
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
        // float dot_product = (cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target)); // * pow(cos(mouseEst.rob_dir - dir_to_target), 2);
        float dot_product = -2*acos((cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target)))/M_PI + 1;
        // turn_rate = PID.calculate(0, mouseEst.rob_dir - dir_to_target, 1);
        turn_rate = PID.calculate(1, dot_product, 1);
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
    if (!goal_found) {
        findGoal(labyrinthEst);
    }
    if (next_points.empty()) {
        point_reached = false;
        if (goal_found) {
            if (!false) { // CONTINUE HERE. Make sure new fastest path is found if it exists.
                stack<vector<int>> temp_path_goal = getPath(labyrinthEst, start_cell, true);
                stack<vector<int>> temp_path_unexplored = getPath(labyrinthEst, start_cell, false);
                shortest_path_length = calcPathLength(temp_path_goal, start_cell);
                float unexplored_path_length = calcPathLength(temp_path_unexplored, start_cell);
                if (shortest_path_length > unexplored_path_length + 1){
                    for(int i; i<temp_path_unexplored.size(); i++) {
                        temp_path_unexplored.pop();
                    }
                    next_points = getPathPoint(labyrinthEst, current_cell, temp_path_unexplored.top());
                } else {
                    fastest_path_found = true;
                }
            } else {
                if (current_cell==start_cell) {
                    next_points = getPath(labyrinthEst, start_cell, true);
                } else {
                    next_points = getPathPoint(labyrinthEst, current_cell, start_cell);
                }
            }
        } else {
            next_points = getPath(labyrinthEst, current_cell, false);
        }
    } else {
        if (point_reached){
            next_points.pop();
            point_reached = false;
            turnPID.reset();
        } else {
            next_point = next_points.top();
            // cout << next_point[0] << ", " << next_point[1] << endl;
            return drive_to(next_point, mouseEst, labyrinthEst, turnPID);
        }

    }
    // cout << current_cell[0] << ", " << current_cell[1] << endl;
    return vector<float>{0, 0};
    
}
  


