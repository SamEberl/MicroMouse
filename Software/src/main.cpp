#include <SDL2/SDL.h>
#include <stdbool.h>
#include <iostream>
#include "draw.h"
#include "classes.h"
#include "defs.h"
#include "utils.h"
#include "estimations.h"

using namespace std;

int main(int argc, char *argv[]) {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("SDL Circle", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
  if (!window) {
    SDL_Log("Unable to create window: %s", SDL_GetError());
    return 1;
  }

  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    SDL_Log("Unable to create renderer: %s", SDL_GetError());
    return 1;
  }

  //SDL wants to have width before height. So to stay consistent it's like this everywhere.
  Robot mouse = Robot((LABYRINTH_WIDTH*CELL_SIZE)-(CELL_SIZE/2), (LABYRINTH_HEIGHT*CELL_SIZE)-(CELL_SIZE/2),
                            M_PI*3/2, DISTANCE_WHEELS, MOUSE_WIDTH, MOUSE_HEIGHT);
  Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT];
  // generate_labyrinth(labyrinth);
  generate_custom_labyrinth(labyrinth);
  print_labyrinth(labyrinth);
  mouse.measureDistances(labyrinth);
  RobotEst mouseEst = RobotEst((LABYRINTH_WIDTH*CELL_SIZE)-(CELL_SIZE/2), (LABYRINTH_HEIGHT*CELL_SIZE)-(CELL_SIZE/2),
                            M_PI*3/2, DISTANCE_WHEELS, MOUSE_WIDTH, MOUSE_HEIGHT);
  CellEst labyrinthEst[LABYRINTH_WIDTH][LABYRINTH_HEIGHT];
  init_labyrinth(labyrinthEst);


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

    drawLabyrinth(renderer, labyrinth);
    drawLabyrinth(renderer, labyrinthEst);
    drawRobo(renderer, mouse);
    drawRobo(renderer, mouseEst);

    // if (mouse.sensS.dist_measure < SENSOR_RANGE*4/5) {
    //   mouse.updatePosition(-0.5, 0.5);
    // } else if (mouse.sensR2.dist_measure < SENSOR_RANGE/2) {
    //   mouse.updatePosition(-0.5, 0.5);
    // } else {
    //   mouse.updatePosition(0.5, 0.5);
    // }

    // if (mouseEst.sensS.dist_measure < SENSOR_RANGE*4/5) {
    //   mouseEst.updatePosition(-0.5, 0.5);
    // } else if (mouseEst.sensR2.dist_measure < SENSOR_RANGE/2) {
    //   mouseEst.updatePosition(-0.5, 0.5);
    // } else {
    //   mouseEst.updatePosition(0.5, 0.5);
    // }


    if (mouse.sensS.dist_measure < SENSOR_RANGE*3/5) {
      mouse.updatePosition(-1.5, 1.5);
    } else if (mouse.sensR2.dist_measure < SENSOR_RANGE*2/5) {
      mouse.updatePosition(-1.5, 1.5);
    } else {
      mouse.updatePosition(1.5, 1.5);
    }

    if (mouseEst.sensS.dist_measure < SENSOR_RANGE*3/5) {
      mouseEst.updatePosition(-1.5, 1.5);
    } else if (mouseEst.sensR2.dist_measure < SENSOR_RANGE*2/5) {
      mouseEst.updatePosition(-1.5, 1.5);
    } else {
      mouseEst.updatePosition(1.5, 1.5);
    }

    // mouse.updatePosition(-0.005, 0.005);
    // mouseEst.updatePosition(-0.005, 0.005);
    // mouse.updatePosition(-0.5, 0.5);
    // mouseEst.updatePosition(-0.5, 0.5);

    mouse.measureDistances(labyrinth);
    mouseEst.compareDistances(renderer, mouse, labyrinthEst);


    cout << "real mouse direction: " << mouse.rob_dir << endl;
    mouseEst.horizontal_dist_to_west_wall(M_PI/6, 1);

    SDL_RenderPresent(renderer);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
