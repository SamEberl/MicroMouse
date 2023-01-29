#include <SDL2/SDL.h>
#include <classes.h>

#ifndef DRAW_H
#define DRAW_H

void drawSquareGrid(SDL_Renderer *renderer, int sideLength);
void drawCircle(SDL_Renderer *renderer, int x, int y, int radius);
void drawSquare(SDL_Renderer *renderer, int x, int y, int w, int h);
void drawRobo(SDL_Renderer *renderer, Robot mouse);

#endif