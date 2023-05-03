#include <SDL2/SDL.h>
#include <stdbool.h>
#include <iostream>
#include "draw.h"
#include "classes.h"
#include "defs.h"
#include "utils.h"
#include "estimations.h"
#include "algorithm.h"

using namespace std;

int main(int argc, char *argv[]) {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("MicroMouse Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
  if (!window) {
    SDL_Log("Unable to create window: %s", SDL_GetError());
    return 1;
  }

  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    SDL_Log("Unable to create renderer: %s", SDL_GetError());
    return 1;
  }

  float rob_x = (LABYRINTH_WIDTH*(CELL_SIZE + WALL_WIDTH))-(CELL_SIZE/2);
  // rob_x = (3*(CELL_SIZE + WALL_WIDTH))-(CELL_SIZE/2);
  float rob_y = (LABYRINTH_HEIGHT*(CELL_SIZE + WALL_WIDTH))-(CELL_SIZE/2);
  // rob_y = (1*(CELL_SIZE + WALL_WIDTH))-(CELL_SIZE/2);
  float rob_dir = M_PI*3/2;
  // rob_dir = M_PI*3.5/2;

  //SDL wants to have width before height. So to stay consistent it's like this everywhere.
  Robot mouse = Robot(rob_x, rob_y, rob_dir, DISTANCE_WHEELS, MOUSE_WIDTH, MOUSE_HEIGHT);
  Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT];
  Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1];
  // generate_labyrinth(labyrinth, corners);
  generate_custom_labyrinth(labyrinth, corners);
  print_labyrinth(labyrinth);
  mouse.measureDistances(labyrinth, corners);

  RobotEst mouseEst = RobotEst(rob_x, rob_y, rob_dir, DISTANCE_WHEELS, MOUSE_WIDTH, MOUSE_HEIGHT);
  CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT];
  CornerEst cornersEst[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1];
  init_labyrinth(labyrinthEst);
  init_corners(cornersEst);

  PIDController controller = PIDController(0.7, 0.05, 0.01, 1.0, -1.0);


  // int grid[LABYRINTH_WIDTH][LABYRINTH_HEIGHT] = {0};
  // vector<int> start = {LABYRINTH_WIDTH-1, LABYRINTH_HEIGHT-1};
  // getPath(labyrinthEst, start, {-1, -1});


  Planner planner;

  bool running = true;
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    drawLabyrinth(renderer, labyrinth, corners);
    drawLabyrinth(renderer, labyrinthEst);
    drawRobo(renderer, mouse);
    drawRobo(renderer, mouseEst);

    // if (mouse.sensS.dist_measure < SENSOR_RANGE*2/5) {
    //   mouse.updatePosition(-0.5, 0.5);
    // } else if (mouse.sensR2.dist_measure < SENSOR_RANGE*2/5) {
    //   mouse.updatePosition(-0.5, 0.5);
    // } else {
    //   mouse.updatePosition(0.5, 0.5);
    // }

    // if (mouseEst.sensS.dist_measure < SENSOR_RANGE*2/5) {
    //   mouseEst.updatePosition(-0.5, 0.5);
    // } else if (mouseEst.sensR2.dist_measure < SENSOR_RANGE*2/5) {
    //   mouseEst.updatePosition(-0.5, 0.5);
    // } else {
    //   mouseEst.updatePosition(0.5, 0.5);
    // }


    // if (mouse.sensS.dist_measure < SENSOR_RANGE/4) {
    //   mouse.updatePosition(-2.5, 2.5);
    // } else if (mouse.sensR2.dist_measure < SENSOR_RANGE/4) {
    //   mouse.updatePosition(-2.5, 2.5);
    // } else {
    //   mouse.updatePosition(2.5, 2.5);
    // }

    // if (mouseEst.sensS.dist_measure < SENSOR_RANGE*2/5) {
    //   mouseEst.updatePosition(-2.5, 2.5);
    // } else if (mouseEst.sensR2.dist_measure < SENSOR_RANGE*2/5) {
    //   mouseEst.updatePosition(-2.5, 2.5);
    // } else {
    //   mouseEst.updatePosition(2.5, 2.5);
    // }

    // mouse.updatePosition(-0.05, 0.05);
    // mouseEst.updatePosition(-0.05, 0.05);
    // mouse.updatePosition(-0.5, 0.5);
    // mouseEst.updatePosition(-0.5, 0.5);

    // float dot = 0*cos(mouseEst.rob_dir) + sin(mouseEst.rob_dir);
    // float driveDiff = controller.calculate(1, dot, 1);
    // mouse.updatePosition(0.3 + driveDiff, 0.3 - driveDiff);

    vector<float> wheel_speed = planner.update(mouseEst, labyrinthEst);
    mouse.updatePosition(wheel_speed[0], wheel_speed[1]);
    
    

    mouse.measureDistances(labyrinth, corners);
    mouseEst.updatePosition(renderer, mouse, labyrinthEst, cornersEst);
    mouseEst.localization();

    SDL_RenderPresent(renderer);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
