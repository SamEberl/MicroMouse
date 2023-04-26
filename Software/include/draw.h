#include <SDL2/SDL.h>
#include "classes.h"
#include "defs.h"
#include "estimations.h"

#ifndef DRAW_H
#define DRAW_H

using namespace std;

void drawSquareGrid(SDL_Renderer *renderer, int sideLength);
void drawCircle(SDL_Renderer *renderer, int x, int y, int radius);
void drawSquare(SDL_Renderer *renderer, int x, int y, int w, int h);
void drawRotatedSquare(SDL_Renderer *renderer, float x, float y, float direction_x, float direction_y, float width, float height);
void drawRobo(SDL_Renderer *renderer, Robot mouse);
void drawRobo(SDL_Renderer *renderer, RobotEst mouse);
void drawSensor(SDL_Renderer *renderer, Sensor sensor);
void drawSensor(SDL_Renderer *renderer, SensorEst sensor);
void drawLabyrinth(SDL_Renderer *renderer, Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
void drawLabyrinth(SDL_Renderer *renderer, CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);

#endif