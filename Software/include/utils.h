#include <vector>
#include <stack>
#include "classes.h"
#include "estimations.h"

#ifndef UTILS_H
#define UTILS_H

using namespace std;

float distBetweenPoints(vector<float> p1, vector<float> p2);
int random_int(int min, int max);
float gaussianNoise(float mean, float stddev);
void init_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
void init_labyrinth(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
void generate_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
void generate_custom_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
int print_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
vector<float> findIntersection(vector<float> starting_point, 
                                vector<float> end_point, 
                                vector<float> wall_point1, 
                                vector<float> wall_point2,
                                float& distance_to_intersection, 
                                bool& intersection_found);


#endif
