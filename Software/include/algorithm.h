#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <queue>

typedef struct {
  float kp;
  float ki;
  float kd;
  float maxOutput;
  float minOutput;
  float lastError;
  float totalError;
} PIDController;

typedef struct {
  CellPos start_cell;
  CellPos current_cell;
  CellPos goal_1;
  CellPos goal_2;
  CellPos goal_3;
  CellPos goal_4;
  Stack* next_points = createStack();
  bool goal_found;
  bool fastest_path_found;
  bool point_reached;
  int shortest_path_length;
} Planner;

void init_Planner(Planner& planner);

void findGoal(Planner& planner, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);

CellPos getPath(Planner& planner, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], CellPos begin, char go_to);

Point drive_to(Planner& planner, PIDController PID, CellPos next, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);

Point update(Planner& planner, PIDController& PID, RobotEst mouseEst, CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);


#endif