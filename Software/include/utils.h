#ifndef UTILS_H
#define UTILS_H

#include "defs.h"

/* Define Point structure */
typedef struct {
    float x;
    float y;
} Point;

typedef struct {
    int x;
    int y;
} CellPos;

typedef struct Node {
    CellPos data;
    struct Node* next;
    struct Node* parent;
} Node;

typedef struct Queue {
    Node* front;
    Node* rear;
} Queue;

typedef struct {
    Node* top;
} Stack;

Queue* createQueue();
int isEmpty(Queue* queue);
void push_queue(Queue* queue, Node* parent, CellPos data);
void pop_queue(Queue* queue);
void swap(Queue* queue1, Queue* queue2);
void destroyQueue(Queue* queue);


Stack* createStack();
int isEmpty(Stack* stack);
void push_stack(Stack* stack, CellPos data);
void pop_stack(Stack* stack);
void swap(Stack* stack1, Stack* stack2);
void destroyStack(Stack* stack);



float distBetweenPoints(Point& p1, Point& p2);
CellPos getCellFromPos(Point& rob_pos);
void findIntersection(Point starting_point, 
                      Point end_point, 
                      Point wall_point1, 
                      Point wall_point2,
                      float* distance_to_intersection, 
                      bool* intersection_found, 
                      Point* intersection_point);

void init_labyrinth(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]);
void init_corners(CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]);



#endif
