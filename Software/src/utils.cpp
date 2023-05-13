#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <stack>
#include <random>
#include <algorithm> 
#include "utils.h"
#include "classes.h"
#include "estimations.h"

using namespace std;

float MAZE_NUMBER = 5;

void setup_sdl(){

}

float distBetweenPoints(vector<float> p1, vector<float> p2){
  // function to get distance between two points
  float dist_x, dist_y;
  dist_x = p2[0] - p1[0];
  dist_y = p2[1] - p1[1];
  return (sqrt(pow(dist_x, 2) + pow(dist_y, 2)));
}

// function to generate a random integer between min and max (inclusive)
int random_int(int min, int max) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> dis(min, max);
    return MAZE_NUMBER;
    return dis(gen);
}

float gaussianNoise(float mean, float stddev) {
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<float> dist(mean, stddev);
    return dist(gen);
}


void init_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    for (int i = 0; i < LABYRINTH_WIDTH; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
            labyrinth[i][j].initialize(i, j);
        }
    }
}

void init_labyrinth(CellEst labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    for (int i = 0; i < LABYRINTH_WIDTH; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
            labyrinth[i][j].initialize(i, j);

        }
    }
}

void init_corners(Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    for (int i = 0; i < LABYRINTH_WIDTH+1; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT+1; j++) {
            corners[i][j].initialize(i, j);
        }
    }
}

void init_corners(CornerEst corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    for (int i = 0; i < LABYRINTH_WIDTH+1; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT+1; j++) {
            corners[i][j].initialize(i, j);
        }
    }
}

// function to generate a random labyrinth using depth-first search algorithm
void generate_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    init_labyrinth(labyrinth);
    init_corners(corners);

    stack<Cell*> stack;
    Cell* current = &labyrinth[0][0];
    stack.push(current);

    while (!stack.empty()) {
        current = stack.top();
        stack.pop();

        vector<Cell*> neighbors;
        int row = current->get_row();
        int col = current->get_col();

        if (row > 0) {
            neighbors.push_back(&labyrinth[row-1][col]);
        }
        if (row < LABYRINTH_WIDTH-1) {
            neighbors.push_back(&labyrinth[row+1][col]);
        }
        if (col > 0) {
            neighbors.push_back(&labyrinth[row][col-1]);
        }
        if (col < LABYRINTH_HEIGHT-1) {
            neighbors.push_back(&labyrinth[row][col+1]);
        }

        shuffle(neighbors.begin(), neighbors.end(), default_random_engine(random_int(0, neighbors.size()-1)));

        for (Cell* neighbor : neighbors) {
            if (neighbor->is_seen()) {
                continue;
            }
            if (neighbor->get_row() < current->get_row()) {
                current->remove_wall('N');
                neighbor->remove_wall('S');
            }
            else if (neighbor->get_row() > current->get_row()) {
                current->remove_wall('S');
                neighbor->remove_wall('N');
            }
            else if (neighbor->get_col() < current->get_col()) {
                current->remove_wall('W');
                neighbor->remove_wall('E');
            }
            else if (neighbor->get_col() > current->get_col()) {
                current->remove_wall('E');
                neighbor->remove_wall('W');
            }
            neighbor->set_seen(true);
            stack.push(neighbor);
        }
    }
    labyrinth[0][1].remove_wall('S');
    labyrinth[1][1].remove_wall('N');
}

void generate_custom_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT], Corner corners[LABYRINTH_WIDTH+1][LABYRINTH_HEIGHT+1]) {
    init_labyrinth(labyrinth);
    init_corners(corners);
    // bool walls_E[6][6] = {  {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0}};
    // bool walls_S[6][6] = {  {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0},
    //                         {0, 0, 0, 0, 0, 0}};

    // bool walls_E[6][6] = {  {1, 1, 1, 1, 0, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 0, 1}};
    // bool walls_S[6][6] = {  {0, 1, 1, 1, 0, 0},
    //                         {0, 1, 1, 1, 0, 0},
    //                         {1, 1, 1, 1, 0, 0},
    //                         {1, 1, 1, 1, 0, 0},
    //                         {1, 1, 1, 1, 0, 0},
    //                         {1, 1, 1, 1, 1, 1}};


    // bool walls_E[6][6] = {  {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1}};
    // bool walls_S[6][6] = {  {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1},
    //                         {1, 1, 1, 1, 1, 1}};

// example maze
    bool walls_E[6][6] = {  {0, 0, 0, 1, 1, 1},
                            {0, 0, 0, 0, 1, 1},
                            {0, 1, 1, 1, 0, 1},
                            {0, 0, 1, 1, 0, 1},
                            {1, 1, 1, 0, 0, 1},
                            {0, 1, 0, 0, 1, 1}};
    bool walls_S[6][6] = {  {1, 1, 0, 1, 0, 0},
                            {0, 0, 0, 0, 1, 1},
                            {1, 1, 0, 0, 1, 0},
                            {0, 1, 0, 0, 0, 1},
                            {0, 0, 0, 1, 0, 0},
                            {1, 1, 1, 1, 1, 1}};

// // example maze multiple paths
    // bool walls_E[6][6] = {  {0, 0, 0, 0, 0, 1},
    //                         {0, 1, 0, 0, 1, 1},
    //                         {0, 0, 0, 1, 0, 1},
    //                         {0, 0, 0, 0, 1, 1},
    //                         {0, 0, 0, 0, 0, 1},
    //                         {0, 0, 0, 0, 1, 1}};
    // bool walls_S[6][6] = {  {0, 1, 1, 1, 1, 0},
    //                         {0, 0, 1, 0, 1, 0},
    //                         {1, 1, 0, 1, 1, 0},
    //                         {0, 1, 1, 0, 1, 0},
    //                         {0, 1, 1, 1, 1, 0},
    //                         {1, 1, 1, 1, 1, 1}};

    for (int i = 0; i < LABYRINTH_HEIGHT; i++) {
        for (int j = 0; j < LABYRINTH_WIDTH; j++) {
            if (!walls_E[i][j]) {
                labyrinth[i][j].remove_wall('E');
                labyrinth[i][j+1].remove_wall('W');
            }
            if (!walls_S[i][j]) {
                labyrinth[i][j].remove_wall('S');
                labyrinth[i+1][j].remove_wall('N');
            }
        }
    }
}

int print_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
    // print the labyrinth
    for (int i = 0; i < LABYRINTH_WIDTH; i++) {
        for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
            if (labyrinth[i][j].has_wall('N')) {
                cout << "+---";
            }
            else {
                cout << "+   ";
            }
        }
        cout << "+" << endl;
        for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
            if (labyrinth[i][j].has_wall('W')) {
                cout << "|   ";
            }
            else {
                cout << "    ";
            }
            if (j == LABYRINTH_HEIGHT-1) {
                cout << "|" << endl;
            }
        }
    }
    for (int j = 0; j < LABYRINTH_HEIGHT; j++) {
        cout << "+---";
    }
    cout << "+" << endl;
    return 0;
}

vector<float> findIntersection(vector<float> starting_point, 
                                vector<float> end_point, 
                                vector<float> wall_point1, 
                                vector<float> wall_point2,
                                float& distance_to_intersection, 
                                bool& intersection_found) {
    float x1 = wall_point1[0];
    float y1 = wall_point1[1];
    float x2 = wall_point2[0];
    float y2 = wall_point2[1];
    float x3 = starting_point[0];
    float y3 = starting_point[1];
    float x4 = end_point[0];
    float y4 = end_point[1];
    // float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    // float eps = 0.001;
    // if (denominator < eps && denominator > -eps) {
    //     // the two lines are parallel, so there is no intersection
    //     intersection_found = false;
    //     return vector<float>{-1.0, -1.0};
    // }
    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    float u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
        // the two lines intersect, so return the intersection point and distance
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        distance_to_intersection = sqrt(pow(x - x3, 2) + pow(y - y3, 2));
        intersection_found = true;
        return vector<float>{x, y};
    } else {
        // the two lines do not intersect, so return (-1, -1) and distance -1
        intersection_found = false;
        distance_to_intersection = -1.0;
        return vector<float>{-1.0, -1.0};
    }
}

vector<int> getCellFromPos(vector<float> rob_pos) {
    vector<int> current_cell = {0, 0};
    current_cell[0] = int(round(((rob_pos[0] - CELL_SIZE/2 - WALL_WIDTH)/(CELL_SIZE + WALL_WIDTH))));
    current_cell[1] = int(round(((rob_pos[1] - CELL_SIZE/2 - WALL_WIDTH)/(CELL_SIZE + WALL_WIDTH))));
    return current_cell;
}
