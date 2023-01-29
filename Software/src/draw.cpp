#include <SDL2/SDL.h>
#include "draw.h"
#include "classes.h"

void drawSquareGrid(SDL_Renderer *renderer, int sideLength)
{
    // Set the color for the grid cell
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    int cellSize = 64;
    for (int i = 0; i <= sideLength; i += 1)
    {
        SDL_RenderDrawLine(renderer, 0, i*cellSize, sideLength*cellSize, i*cellSize);
        SDL_RenderDrawLine(renderer, i*cellSize, 0, i*cellSize, sideLength*cellSize);
    }
}

void drawCircle(SDL_Renderer *renderer, int x, int y, int radius) {
    for (int i = 0; i < 360; i++) {
        float degInRad = i * M_PI / 180;
        for (int j = radius; j > 0; j--) {
            SDL_RenderDrawPoint(renderer, x + cos(degInRad) * j, y + sin(degInRad) * j);
        }
    }
}

void drawSquare(SDL_Renderer *renderer, int x, int y, int w, int h)
{
  SDL_Rect rect = {x, y, w, h};
  SDL_RenderFillRect(renderer, &rect);
}

void drawRobo(SDL_Renderer *renderer, Robot mouse){
    float x = mouse.position[0];
    float y = mouse.position[1];
    float x_dir = mouse.direction[0];
    float y_dir = mouse.direction[1];
    float radius = mouse.radius;
    SDL_SetRenderDrawColor(renderer, 55, 55, 55, SDL_ALPHA_OPAQUE);
    drawCircle(renderer, x, y, radius);

    SDL_SetRenderDrawColor(renderer, 255, 55, 55, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer, x, y, x+x_dir*radius, y+y_dir*radius);
}