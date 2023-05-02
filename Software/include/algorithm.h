#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <queue>

class PIDController {
  public:
    PIDController();
    PIDController(float kp, float ki, float kd, float maxOutput, float minOutput);
    float calculate(float setPoint, float processVariable, float deltaTime);
    void reset();

    float kp;
    float ki;
    float kd;
    float maxOutput;
    float minOutput;
    float lastError;
    float totalError;
    
};


class Planner {
public:
  Planner();
  vector<float> update(RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
  vector<float> drive_to(vector<int> next_point, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], PIDController PID);
  PIDController turnPID;
  vector<int> start_cell;
  vector<int> current_cell;
  vector<int> goal_cell;
  vector <int> next_point;
  stack<vector<int>> next_points;
  bool goal_found;
  bool point_reached;
};

bool findGoal(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<float> goal_cell);
void floodFill(int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start);
stack<vector<int>> getPath(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start, vector<int> goal);
void printPath(queue<vector<int>> path);
void printPath(stack<vector<int>> path);

#endif