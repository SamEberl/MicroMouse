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

  int start_x = LABYRINTH_WIDTH-1;
  int start_y = LABYRINTH_HEIGHT-1;

  float rob_x = (start_x*(CELL_SIZE + WALL_WIDTH))+(CELL_SIZE/2)+WALL_WIDTH;
  float rob_y = (start_y*(CELL_SIZE + WALL_WIDTH))+(CELL_SIZE/2)+WALL_WIDTH;
  float rob_dir = M_PI*3/2;


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
  Planner planner(start_x, start_y);

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
