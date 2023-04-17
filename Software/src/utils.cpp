#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <stack>
#include <random>
#include <algorithm> 
#include "utils.h"
#include "classes.h"

using namespace std;


float distBetweenPoints(vector<float> p1, vector<float> p2){
  // function to get distance between two points
  float dist_x, dist_y;
  dist_x = p2[0] - p1[0];
  dist_y = p2[1] - p1[1];
  return (sqrt(pow(dist_x, 2) + pow(dist_y, 2)));
}

vector<float> lineLineIntersection(vector<float> l1p1, vector<float> l1p2, vector<float> l2p1, vector<float> l2p2)
{
    // Line AB represented as a1x + b1y = c1
    float a1 = l1p2[1] - l1p1[1];
    float b1 = l1p1[0] - l1p2[0];
    float c1 = a1*(l1p1[0]) + b1*(l1p1[1]);
 
    // Line CD represented as a2x + b2y = c2
    float a2 = l2p2[1] - l2p1[1];
    float b2 = l2p1[0] - l2p2[0];
    float c2 = a2*(l2p1[0])+ b2*(l2p1[1]);
 
    float determinant = a1*b2 - a2*b1;
 
    if (determinant == 0)
    {
        // The lines are parallel. This is simplified
        // by returning a pair of FLT_MAX
        return vector<float> {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    }
    else
    {
        float x = (b2*c1 - b1*c2)/determinant;
        float y = (a1*c2 - a2*c1)/determinant;
        return vector<float> {x, y};
    }
}


bool doIntersect(vector<float> l1p1, vector<float> l1p2, vector<float> l2p1, vector<float> l2p2) {
    float d1, d2, d3, d4;
    vector<float> intersection;
    float distance;

    d1 = crossProduct(l2p1, l2p2, l1p1);
    d2 = crossProduct(l2p1, l2p2, l1p2);
    d3 = crossProduct(l1p1, l1p2, l2p1);
    d4 = crossProduct(l1p1, l1p2, l2p2);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        intersection = intersectionPoint(l1p1, l1p2, l2p1, l2p2);
        // Calculate the distance between the intersection point and the first endpoint of the first line segment
        distance = sqrt(pow(l1p1[0]-intersection[0], 2) + pow(l1p1[1]-intersection[1], 2));
        // Return true if the distance is not zero (i.e. the intersection is not at an endpoint)
        return distance > 0;
    }
    return false;
}

vector<float> intersectionPoint(vector<float> l1p1, vector<float> l1p2, vector<float> l2p1, vector<float> l2p2) {
    vector<float> intersection;
    float a1, b1, c1, a2, b2, c2;
    float determinant;

    a1 = l1p2[1] - l1p1[1];
    b1 = l1p1[0] - l1p2[0];
    c1 = a1 * l1p1[0] + b1 * l1p1[1];

    a2 = l2p2[1] - l2p1[1];
    b2 = l2p1[0] - l2p2[0];
    c2 = a2 * l2p1[0] + b2 * l2p1[1];

    determinant = a1 * b2 - a2 * b1;

    if (determinant == 0) {
        return vector<float> {NAN, NAN};
    }

    intersection[0] = (b2 * c1 - b1 * c2) / determinant;
    intersection[1] = (a1 * c2 - a2 * c1) / determinant;

    return intersection;
}

float crossProduct(vector<float> p, vector<float> q, vector<float> r) {
    return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
}

// function to generate a random integer between min and max (inclusive)
int random_int(int min, int max) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> dis(min, max);
    return dis(gen);
}

// function to generate a random labyrinth using depth-first search algorithm
void generate_labyrinth(Cell labyrinth[LABYRINTH_WIDTH][LABYRINTH_HEIGHT]) {
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
            if (neighbor->is_visited()) {
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
            neighbor->set_visited(true);
            stack.push(neighbor);
        }
    }
}

// function to generate a random labyrinth using depth-first search algorithm
void generate_labyrinthArch(vector<vector<Cell>>& grid) {
    stack<Cell*> stack;
    Cell* current = &grid[0][0];
    stack.push(current);

    while (!stack.empty()) {
        current = stack.top();
        stack.pop();

        vector<Cell*> neighbors;
        int row = current->get_row();
        int col = current->get_col();

        if (row > 0) {
            neighbors.push_back(&grid[row-1][col]);
        }
        if (row < grid.size()-1) {
            neighbors.push_back(&grid[row+1][col]);
        }
        if (col > 0) {
            neighbors.push_back(&grid[row][col-1]);
        }
        if (col < grid[0].size()-1) {
            neighbors.push_back(&grid[row][col+1]);
        }

        shuffle(neighbors.begin(), neighbors.end(), default_random_engine(random_int(0, neighbors.size()-1)));

        for (Cell* neighbor : neighbors) {
            if (neighbor->is_visited()) {
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
            neighbor->set_visited(true);
            stack.push(neighbor);
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

int print_labyrinthArch(vector<vector<Cell>> labyrinth) {
    // print the labyrinth
    for (int i = 0; i < labyrinth.size(); i++) {
        for (int j = 0; j < labyrinth[i].size(); j++) {
            if (labyrinth[i][j].has_wall('N')) {
                cout << "+---";
            }
            else {
                cout << "+   ";
            }
        }
        cout << "+" << endl;
        for (int j = 0; j < labyrinth[i].size(); j++) {
            if (labyrinth[i][j].has_wall('W')) {
                cout << "|   ";
            }
            else {
                cout << "    ";
            }
            if (j == labyrinth[i].size()-1) {
                cout << "|" << endl;
            }
        }
    }
    for (int j = 0; j < labyrinth[0].size(); j++) {
        cout << "+---";
    }
    cout << "+" << endl;
    return 0;
}

vector<float> findIntersection(SDL_Renderer *renderer,
                                vector<float> starting_point, 
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
    float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    float eps = 0.001;
    if (denominator < eps && denominator > -eps) {
        // the two lines are parallel, so there is no intersection
        intersection_found = false;
        return vector<float>{-1.0, -1.0};
    }
    // float t1 = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / denominator;
    // float t2 = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / denominator;
    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    float u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    if (0 <= t && t <= 1 && 0 <= u && u <= 1) {
        // the two lines intersect, so return the intersection point and distance
        float x = x1 + t * (x2 - x1);
        float y = y1 + t * (y2 - y1);
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 128);
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 128);
        SDL_RenderDrawPoint(renderer, x, y);
        distance_to_intersection = sqrt(pow(x - x3, 2) + pow(y - y3, 2));
        intersection_found = true;
        return vector<float>{x, y};
    } else {
        // the two lines do not intersect, so return (-1, -1) and distance 0
        intersection_found = false;
        distance_to_intersection = 0.0;
        return vector<float>{-1.0, -1.0};
    }
}

vector<float> findIntersectionArchArch(SDL_Renderer *renderer,
                                vector<float> starting_point, 
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
    float denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    float eps = 0.001;
    if (denominator < eps && denominator > -eps) {
        // the two lines are parallel, so there is no intersection
        intersection_found = false;
        return vector<float>{-1.0, -1.0};
    }
    float t1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator;
    float t2 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator;
    if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
        // the two lines intersect, so return the intersection point and distance
        float x = x1 + t2 * (x2 - x1);
        float y = y1 + t2 * (y2 - y1);
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 128);
        SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 128);
        SDL_RenderDrawPoint(renderer, x, y);
        distance_to_intersection = sqrt(pow(x - x3, 2) + pow(y - y3, 2));
        intersection_found = true;
        return vector<float>{x, y};
    } else {
        // the two lines do not intersect, so return (-1, -1) and distance 0
        intersection_found = false;
        distance_to_intersection = 0.0;
        return vector<float>{-1.0, -1.0};
    }
}

vector<float> findIntersectionArch(vector<float> starting_point, vector<float> end_point, vector<float> wall_point1, vector<float> wall_point2) {
    float x1 = wall_point1[0];
    float y1 = wall_point1[1];
    float x2 = wall_point2[0];
    float y2 = wall_point2[1];
    float x3 = starting_point[0];
    float y3 = starting_point[1];
    float x4 = end_point[0];
    float y4 = end_point[1];
    float denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (denominator == 0) {
        // the two lines are parallel, so there is no intersection
        return vector<float>{-1.0, -1.0};
    }
    float t1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator;
    float t2 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator;
    if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
        // the two lines intersect, so return the intersection point
        float x = x1 + t2 * (x2 - x1);
        float y = y1 + t2 * (y2 - y1);
        return vector<float>{x, y};
    } else {
        // the two lines do not intersect, so return (-1, -1)
        return vector<float>{-1.0, -1.0};
    }
}


/*
int main() {
    LineSegment l1 = {{0, 0}, {1, 1}};
    LineSegment l2 = {{0, 1}, {1, 0}};
    Point intersection;

    if (doIntersect(l1, l2, intersection)) {
        float distance = sqrt(pow(intersection.x - l1.start.x, 2) + pow(intersection.y - l1.start.y, 2));
        std::cout << "Intersection at (" << intersection.x << ", " << intersection.y <<



vector<int> multiply(const vector<vector<int>>& matrix, 
                          const vector<int>& vector) {
  int rows = matrix.size();
  int cols = matrix[0].size();

  vector<int> result(rows);
  for (int i = 0; i < rows; i++) {
    int sum = 0;
    for (int j = 0; j < cols; j++) {
      sum += matrix[i][j] * vector[j];
    }
    result[i] = sum;
  }

  return result;
}

int test() {
  vector<vector<int>> matrix = {{1, 2, 3}, 
                                          {4, 5, 6}, 
                                          {7, 8, 9}};
  vector<int> vector = {1, 2, 3};
  vector<int> result = multiply(matrix, vector);

  for (const auto& item : result) {
    std::cout << item << " ";
  }

  return 0;
}
*/

