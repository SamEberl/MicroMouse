#include <SDL2/SDL.h>
#include <stdbool.h>
#include <iostream>
#include "draw.h"
#include "classes.h"
#include "defs.h"
#include "utils.h"

using namespace std;

int main(int argc, char *argv[]) {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("SDL Circle", SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT,
                                        SDL_WINDOW_SHOWN);
  if (!window) {
    SDL_Log("Unable to create window: %s", SDL_GetError());
    return 1;
  }

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer) {
    SDL_Log("Unable to create renderer: %s", SDL_GetError());
    return 1;
  }

  //SDL wants to have width before height. So to stay consistent it's like this everywhere.
  Robot mouse = Robot((LABYRINTH_WIDTH*CELL_SIZE)-(CELL_SIZE/2), (LABYRINTH_HEIGHT*CELL_SIZE)-(CELL_SIZE/2),
                            0, -1, DISTANCE_WHEELS, MOUSE_WIDTH, MOUSE_HEIGHT);
  Sensor sensor1 = Sensor(mouse, M_PI/2, mouse.height/2);
  Sensor sensor2 = Sensor(mouse, M_PI/4, mouse.height/2);
  Sensor sensor3 = Sensor(mouse, 0.0, mouse.height/2);
  Sensor sensor4 = Sensor(mouse, -M_PI/4, mouse.height/2);
  Sensor sensor5 = Sensor(mouse, -M_PI/2, mouse.height/2);

  // vector<vector<Cell>> labyrinth(LABYRINTH_WIDTH, vector<Cell>(LABYRINTH_HEIGHT, Cell(0, 0)));
  Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT];
  for (int i = 0; i < LABYRINTH_WIDTH; i++) {
      for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
          labyrinth[i][j].initialize(i, j);
      }
  }
  generate_labyrinth(labyrinth);
  print_labyrinth(labyrinth);

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

    //SDL_SetRenderDrawColor(renderer, 255, 0, 0, 0);
    //drawSquareGrid(renderer, 16);

    drawRobo(renderer, mouse);
    
    // drawSensor(renderer, mouse, sensor1);
    // drawSensor(renderer, mouse, sensor2);
    // drawSensor(renderer, mouse, sensor3);
    // drawSensor(renderer, mouse, sensor4);
    drawSensor(renderer, mouse, sensor5);
    

    drawLabyrinth(renderer, labyrinth);
    mouse.updatePosition(0.3, 0.31);
    sensor1.updatePosition(mouse);
    sensor2.updatePosition(mouse);
    sensor3.updatePosition(mouse);
    sensor4.updatePosition(mouse);
    sensor5.updatePosition(mouse);
    // sensor1.getDistanceToWall(labyrinth);
    // sensor2.getDistanceToWall(labyrinth);
    // sensor3.getDistanceToWall(labyrinth);
    // sensor4.getDistanceToWall(labyrinth);
    sensor5.getDistanceToWall(renderer, labyrinth);
    cout << "intersectionDistMeasure:  " << sensor5.dist_measure << ", " << " " << endl;


    SDL_RenderPresent(renderer);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
