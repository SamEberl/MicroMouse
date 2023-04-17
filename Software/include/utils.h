#include <vector>
#include <stack>
#include "classes.h"

#ifndef UTILS_H
#define UTILS_H

using namespace std;

float distBetweenPoints(vector<float> p1, vector<float> p2);
vector<float> lineLineIntersection(vector<float> l1p1, vector<float> l1p2, vector<float> l2p1, vector<float> l2p2);
bool doIntersect(std::vector<float> l1p1, vector<float> l1p2, vector<float> l2p1, vector<float> l2p2);
vector<float> intersectionPoint(vector<float> l1p1, vector<float> l1p2, vector<float> l2p1, vector<float> l2p2);
float crossProduct(std::vector<float> p, vector<float> q, vector<float> r);
int random_int(int min, int max);
void generate_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
int print_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
vector<float> findIntersection(SDL_Renderer *renderer,
                                vector<float> starting_point, 
                                vector<float> end_point, 
                                vector<float> wall_point1, 
                                vector<float> wall_point2,
                                float& distance_to_intersection, 
                                bool& intersection_found);


#endif
