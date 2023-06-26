#include <stdio.h>
#include <math.h>
#include "utils.h"
#include "estimations.h"

#include <stdlib.h>

Queue* createQueue() {
    Queue* queue = (Queue*)malloc(sizeof(Queue));
    queue->front = NULL;
    queue->rear = NULL;
    return queue;
}

int isEmpty(Queue* queue) {
    return (queue->front == NULL);
}

void push_queue(Queue* queue, Node* parent, CellPos data) {
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->data = data;
    newNode->next = NULL;
    newNode->parent = parent;

    if (isEmpty(queue)) {
        queue->front = newNode;
        queue->rear = newNode;
    } else {
        queue->rear->next = newNode;
        queue->rear = newNode;
    }
}

void pop_queue(Queue* queue) {
    if (isEmpty(queue)) {
        printf("Queue is empty!\n");
    }

    Node* temp = queue->front;
    // CellPos data = temp->data;
    queue->front = queue->front->next;

    if (queue->front == NULL) {
        queue->rear = NULL;
    }

    free(temp);
}

void swap(Queue* queue1, Queue* queue2) {
    Queue tempQueue = *queue1;
    *queue1 = *queue2;
    *queue2 = tempQueue;
}

void destroyQueue(Queue* queue) {
    while (!isEmpty(queue)) {
        pop_queue(queue);
    }
    free(queue);
}




Stack* createStack() {
    Stack* stack = (Stack*)malloc(sizeof(Stack));
    stack->top = NULL;
    return stack;
}

int isEmpty(Stack* stack) {
    return (stack->top == NULL);
}

void push_stack(Stack* stack, CellPos data) {
    Node* newNode = (Node*)malloc(sizeof(Node));
    newNode->data = data;
    newNode->next = stack->top;
    stack->top = newNode;
}

void pop_stack(Stack* stack) {
    if (isEmpty(stack)) {
        printf("Stack is empty!\n");
        return;
    }
    Node* temp = stack->top;
    stack->top = stack->top->next;
    free(temp);
}

void swap(Stack* stack1, Stack* stack2) {
    Stack tempStack = *stack1;
    *stack1 = *stack2;
    *stack2 = tempStack;
}

void destroyStack(Stack* stack) {
    while (!isEmpty(stack)) {
        pop_stack(stack);
    }
    free(stack);
}



float distBetweenPoints(Point& p1, Point& p2){
  // function to get distance between two points
  float dist_x, dist_y;
  dist_x = p2.x - p1.x;
  dist_y = p2.y - p1.y;
  return (sqrt(pow(dist_x, 2) + pow(dist_y, 2)));
}


CellPos getCellFromPos(Point& pos) {
    CellPos current_cell;
    current_cell.x = int(round(((pos.x - CELL_SIZE/2 - WALL_WIDTH)/(CELL_SIZE + WALL_WIDTH))));
    current_cell.y = int(round(((pos.y - CELL_SIZE/2 - WALL_WIDTH)/(CELL_SIZE + WALL_WIDTH))));
    return current_cell;
}


void findIntersection(Point starting_point, 
                      Point end_point, 
                      Point wall_point1, 
                      Point wall_point2,
                      float* distance_to_intersection, 
                      bool* intersection_found, 
                      Point* intersection_point) {

    float x1 = wall_point1.x;
    float y1 = wall_point1.y;
    float x2 = wall_point2.x;
    float y2 = wall_point2.y;
    float x3 = starting_point.x;
    float y3 = starting_point.y;
    float x4 = end_point.x;
    float y4 = end_point.y;

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    float u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

    if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
        /* the two lines intersect, so return the intersection point and distance */
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        *distance_to_intersection = sqrt(pow(x - x3, 2) + pow(y - y3, 2));
        *intersection_found = true;
        intersection_point->x = x;
        intersection_point->y = y;
    } else {
        /* the two lines do not intersect, so return (-1, -1) and distance -1 */
        *intersection_found = false;
        *distance_to_intersection = -1.0;
        intersection_point->x = -1.0;
        intersection_point->y = -1.0;
    }
}


void init_labyrinth(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    for (int i = 0; i < LABYRINTH_WIDTH; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
            CellEst_initialize(&labyrinth[i][j], i, j);
            // labyrinth[i][j].initialize(i, j);

        }
    }
}

void init_corners(CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    for (int i = 0; i < LABYRINTH_WIDTH+1; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT+1; j++) {
            CornerEst_initialize(&corners[i][j], i, j);
            // corners[i][j].initialize(i, j);
        }
    }
}

