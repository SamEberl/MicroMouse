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

#include <math.h>

float KP = 2;
float KI = 1;
float KD = 1;
float PID_MAX = 1.0;
float PID_MIN = -1.0;
float THRESHOLD_NO_WALL = 0.2;


float PID_calculate(PIDController PID, float setPoint, float processVariable, float deltaTime) {
    float error = setPoint - processVariable;
    float derivative = (error - PID.lastError) / deltaTime;
    PID.totalError += error * deltaTime;
    float output = PID.kp * error + PID.ki * PID.totalError + PID.kd * derivative;
    output = fminf(output, PID.maxOutput);
    output = fmaxf(output, PID.minOutput);
    PID.lastError = error;
    return output;
}

void init_Planner(Planner& planner) {
    planner.start_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    planner.current_cell = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
    planner.goal_found = false;
    planner.point_reached = false;
    planner.shortest_path_length = 99999;
};

void findGoal(Planner& planner, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    for (int x = 1; x < LABYRINTH_WIDTH; x++) {
        for (int y = 1; y < LABYRINTH_HEIGHT; y++) {
            if( labyrinth[y][x].N < THRESHOLD_NO_WALL && 
                labyrinth[y][x].W < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].S < THRESHOLD_NO_WALL &&
                labyrinth[y-1][x-1].E < THRESHOLD_NO_WALL) {

                planner.goal_1 = {x-1, y-1};
                planner.goal_2 = {x, y-1};
                planner.goal_3 = {x, y};
                planner.goal_4 = {x-1, y};
                
                planner.goal_found = true;
            }
        }
    }
}

CellPos getPath(Planner& planner, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CellPos begin, char go_to) {
    // Finds path from begin to start (S), Goal (G) or Unexplored (U) and saves it in next_points. Also returns the cell the algorithm drives to.
    int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
    // Define a queue to store the cells to be checked
    Queue* q = createQueue();
    Queue* q_next = createQueue();
    Node current;
    Node cur_parent;
    Node waypoint;
    CellPos driving_to;
    CellPos next;
    push_queue(q, NULL, begin);
    bool done = false;
    // Loop until the end is reached or the queue is empty
    while (!isEmpty(q)) {
        // Get the next cell from the queue
        current = *q->front;
        pop_queue(q);

        switch (go_to)
        {
        case 'S':
            if ((current.data.x == planner.start_cell.x) && (current.data.y == planner.start_cell.y)){
                done = true;
            }
            break;
        case 'G':
            if (((current.data.x == planner.goal_1.x) && (current.data.y == planner.goal_1.y)) || 
                ((current.data.x == planner.goal_2.x) && (current.data.y == planner.goal_2.y)) || 
                ((current.data.x == planner.goal_3.x) && (current.data.y == planner.goal_3.y)) || 
                ((current.data.x == planner.goal_4.x) && (current.data.y == planner.goal_4.y))){
                done = true;
            }
            break;
        case 'U':
            if (!labyrinth[current.data.y][current.data.x].is_seen){
                done = true;
            }
            break;
        default:
            break;
        }

        // Check if the current cell is the end
        if (done) {
            driving_to = current.data;
            // Construct the path from the end to the begin
            if ((current.data.x != begin.x) || (current.data.y != begin.y)) {
                waypoint = current;
                planner.next_points.push(current);
                current = *current.parent;
                cur_parent = *current.parent;
                while ((current.data.x != begin.x) || (current.data.y != begin.y)) {
                    if ((waypoint.data.x == current.data.x && current.data.x == cur_parent.data.x) || (waypoint.data.y == current.data.y && current.data.y == cur_parent.data.y)) {
                        current = *current.parent;
                    } else {
                        waypoint = current;
                        planner.next_points.push(current);
                        current = *current.parent;
                    }
                }
            } else {
                planner.next_points.push(current);
            }
            planner.point_reached = false;
            return driving_to;
        }
        // Check the north cell
        if ((current.data.y-1) >= 0 && grid[current.data.y-1][current.data.x] == 0) {
            if (current.data.x!=begin.x || (current.data.y-1)!=begin.y){
                if (labyrinth[current.data.y][current.data.x].N < 0.2) {
                    next = {current.data.x, current.data.y-1};
                    push_queue(q_next, &current, next);
                }
            }
        }
        // Check the east cell
        if (current.data.x < LABYRINTH_WIDTH-1 && grid[current.data.y][current.data.x+1] == 0) {
            if ((current.data.x+1)!=begin.x || current.data.y!=begin.y){
                if (labyrinth[current.data.y][current.data.x].E < 0.2) {
                    next = {current.data.x+1, current.data.y};
                    push_queue(q_next, &current, next);
                }
            }
        }
        // Check the south cell
        if (current.data.y < LABYRINTH_HEIGHT-1 && grid[current.data.y+1][current.data.x] == 0) {
            if (current.data.x!=begin.x || (current.data.y+1)!=begin.y){
                if (labyrinth[current.data.y][current.data.x].S < 0.2) {
                    next = {current.data.x, current.data.y+1};
                    push_queue(q_next, &current, next);
                }
            }
        }
        // Check the west cell
        if ((current.data.x-1) >= 0 && grid[current.data.y][current.data.x-1] == 0) {
            if ((current.data.x-1)!=begin.x || current.data.y!=begin.y){
                if (labyrinth[current.data.y][current.data.x].W < 0.2) {
                    next = {current.data.x-1, current.data.y};
                    push_queue(q_next, &current, next);
                }
            }
        }
        // Mark the current cell with the counter
        grid[current.data.y][current.data.x] = 1;
        // Get the next queue
        if (isEmpty(q)) {
            swap(q, q_next);
        }
    }
    return CellPos {-1, -1};
} 

float calcPathLength(stack<CellPos> path, CellPos start) {
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

void printPath(stack<CellPos> path) {
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


Point drive_to(Planner& planner, PIDController PID, CellPos next, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]){
    float turn_rate;
    float target_tolerance = 25;
    float dir_tolerance = M_PI/60; //3 degrees to either side

    Point target = {-1.0, -1.0};
    target.x = float(planner.next_points.top().x) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;
    target.y = float(planner.next_points.top().y) * (CELL_SIZE + WALL_WIDTH) + CELL_SIZE/2 + WALL_WIDTH;

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
        labyrinthEst[planner.next_points.top().y][planner.next_points.top().x].is_seen = true;
        planner.point_reached = true;
        return Point{0.0, 0.0};
    } else if (abs(mouseEst.rob_dir - dir_to_target) < dir_tolerance) {
        // drive straight
        return Point{PID.maxOutput, PID.maxOutput};
    } else {
        // float dot_product = -2*acos((cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target)))/M_PI + 1;
        float dot_product = -acos((cos(mouseEst.rob_dir)*cos(dir_to_target) + sin(mouseEst.rob_dir)*sin(dir_to_target)));

        turn_rate = PID_calculate(PID, 0, dot_product, 1);
        // if (turn_rate < 0){
        //     turn_rate *= -1;
        // }
        float speed_forward = (PID.maxOutput-turn_rate)/2;
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


Point update(Planner& planner, PIDController& PID, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    planner.current_cell = getCellFromPos(mouseEst.rob_pos);
    if (planner.next_points.empty()) {
        if (!planner.goal_found) {
            findGoal(planner, labyrinthEst);
        }
        if (planner.goal_found) {
            if (planner.fastest_path_found) {
                if ((planner.current_cell.x == planner.start_cell.x) && (planner.current_cell.y == planner.start_cell.y)) {
                    getPath(planner, labyrinthEst, planner.start_cell, 'G');
                } else {
                    getPath(planner, labyrinthEst, planner.current_cell, 'S');
                }
            } else {
                stack<CellPos> goal_path;
                getPath(planner, labyrinthEst, planner.start_cell, 'G');
                planner.shortest_path_length = calcPathLength(planner.next_points, planner.start_cell);
                swap(planner.next_points, goal_path);

                CellPos next_unexplored;
                next_unexplored = getPath(planner, labyrinthEst, planner.current_cell, 'U');
                float unexplored_path_length = calcPathLength(planner.next_points, planner.current_cell);
                int manhatten_to_start = abs(next_unexplored.x-planner.start_cell.x)+abs(next_unexplored.y-planner.start_cell.y);

                if(planner.shortest_path_length <= (unexplored_path_length + manhatten_to_start + 1)){
                    planner.fastest_path_found = true;
                    stack<CellPos> empty;
                    swap(planner.next_points, empty);
                } 
            }
        } else {
            getPath(planner, labyrinthEst, planner.current_cell, 'U');
        }
        if (!planner.next_points.empty()) {
            printPath(planner.next_points);
        }
    } else {
        if (planner.point_reached){
            planner.next_points.pop();
            planner.point_reached = false;
        } else {
            return drive_to(planner, PID, planner.next_points.top(), mouseEst, labyrinthEst);
        }

    }
    return Point{0, 0};
}
  


