#include <SDL2/SDL.h>
#include <cmath>
#include <iostream>
#include "draw.h"
#include "classes.h"
#include "defs.h"

void drawSquareGrid(SDL_Renderer *renderer, int sideLength)
{
    // Set the color for the grid cell
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    for (int i = 0; i <= sideLength; i += 1)
    {
        SDL_RenderDrawLine(renderer, 0, i*CELL_SIZE, sideLength*CELL_SIZE, i*CELL_SIZE);
        SDL_RenderDrawLine(renderer, i*CELL_SIZE, 0, i*CELL_SIZE, sideLength*CELL_SIZE);
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

void drawRotatedSquare(SDL_Renderer *renderer, float x, float y, float direction_x, float direction_y, float width, float height) {
    float alpha = atan2(direction_y, direction_x);
    float ca = cos(alpha);
    float sa = sin(alpha);
    int x1, y1, x2, y2, x3, y3, x4, y4;
    x1 = ca*(width/2) + -sa*(height/2) + x;
    y1 = sa*(width/2) + ca*(height/2) + y;
    x2 = ca*(width/2) + -sa*(-height/2) + x;
    y2 = sa*(width/2) + ca*(-height/2) + y;
    x3 = ca*(-width/2) + -sa*(-height/2) + x;
    y3 = sa*(-width/2) + ca*(-height/2) + y;
    x4 = ca*(-width/2) + -sa*(height/2) + x;
    y4 = sa*(-width/2) + ca*(height/2) + y;
    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
    SDL_RenderDrawLine(renderer, x2, y2, x3, y3);
    SDL_RenderDrawLine(renderer, x3, y3, x4, y4);
    SDL_RenderDrawLine(renderer, x4, y4, x1, y1);
}


void drawRobo(SDL_Renderer *renderer, Robot mouse){
    float x = mouse.position[0];
    float y = mouse.position[1];
    float x_dir = cos(mouse.direction);
    float y_dir = sin(mouse.direction);
    float width = mouse.width;
    float height = mouse.height;
    SDL_SetRenderDrawColor(renderer, 55, 55, 55, SDL_ALPHA_OPAQUE);
    drawRotatedSquare(renderer, x, y, x_dir, y_dir, width, height);
    SDL_SetRenderDrawColor(renderer, 255, 55, 55, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer, x, y, x+x_dir*width/2, y+y_dir*height/2);
}

void drawSensor(SDL_Renderer *renderer, Robot mouse, Sensor sensor){
    float x = sensor.position[0];
    float y = sensor.position[1];
    SDL_SetRenderDrawColor(renderer, 50, 255, 50, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawLine(renderer, x, y,
                        x + cos(mouse.direction + sensor.offset_direction)*sensor.dist_measure, 
                        y + sin(mouse.direction + sensor.offset_direction)*sensor.dist_measure);
}


void drawLabyrinth(SDL_Renderer *renderer, Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]){
    std::vector<float> p1(2), p2(2), p3(2), p4(2);

    for (int i = 0; i < LABYRINTH_WIDTH; i += 1)
    {
        for (int j = 0; j < LABYRINTH_HEIGHT; j += 1){
            p1 = labyrinth[i][j].p1;
            p2 = labyrinth[i][j].p2;
            p3 = labyrinth[i][j].p3;
            p4 = labyrinth[i][j].p4;

            //draw north
            if (!labyrinth[i][j].north_seen){
                SDL_SetRenderDrawColor(renderer, 200, 200, 255, 128);
                SDL_RenderDrawLine(renderer, p1[0]*CELL_SIZE, p1[1]*CELL_SIZE, p2[0]*CELL_SIZE, p2[1]*CELL_SIZE);
            } else if (labyrinth[i][j].north_exists){
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderDrawLine(renderer, p1[0]*CELL_SIZE, p1[1]*CELL_SIZE, p2[0]*CELL_SIZE, p2[1]*CELL_SIZE);
            }
            

            //draw east
            if (!labyrinth[i][j].east_seen){
                SDL_SetRenderDrawColor(renderer, 200, 200, 255, 128);
                SDL_RenderDrawLine(renderer, p2[0]*CELL_SIZE, p2[1]*CELL_SIZE, p3[0]*CELL_SIZE, p3[1]*CELL_SIZE);
            } else if (labyrinth[i][j].east_exists){
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderDrawLine(renderer, p2[0]*CELL_SIZE, p2[1]*CELL_SIZE, p3[0]*CELL_SIZE, p3[1]*CELL_SIZE);
            }
            

            //draw south
            if (!labyrinth[i][j].south_seen){
                SDL_SetRenderDrawColor(renderer, 200, 200, 255, 128);
                SDL_RenderDrawLine(renderer, p4[0]*CELL_SIZE, p4[1]*CELL_SIZE, p3[0]*CELL_SIZE, p3[1]*CELL_SIZE);
            } else if (labyrinth[i][j].south_exists){
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderDrawLine(renderer, p4[0]*CELL_SIZE, p4[1]*CELL_SIZE, p3[0]*CELL_SIZE, p3[1]*CELL_SIZE);
            }
            

            //draw west
            if (!labyrinth[i][j].west_seen){
                SDL_SetRenderDrawColor(renderer, 200, 200, 255, 128);
                SDL_RenderDrawLine(renderer, p1[0]*CELL_SIZE, p1[1]*CELL_SIZE, p4[0]*CELL_SIZE, p4[1]*CELL_SIZE);
            } else if (labyrinth[i][j].west_exists){
                SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
                SDL_RenderDrawLine(renderer, p1[0]*CELL_SIZE, p1[1]*CELL_SIZE, p4[0]*CELL_SIZE, p4[1]*CELL_SIZE);
            }
        }
    }
}