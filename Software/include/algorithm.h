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
  void findGoal(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
  stack<vector<int>> getPath(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start, bool to_goal);
  stack<vector<int>> getPathPoint(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], vector<int> start, vector<int> end);
  float calcPathLength(stack<vector<int>> path, vector<int> start);
  void printPath(stack<vector<int>> path);
  vector<float> drive_to(vector<int> next_point, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], PIDController PID);
  vector<float> update(RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);

  PIDController turnPID;
  vector<int> start_cell;
  vector<int> current_cell;
  vector <int> next_point;
  stack<vector<int>> next_points;
  bool goal_found;
  bool fastest_path_found;
  bool point_reached;
  float shortest_path_length;
};



#endif