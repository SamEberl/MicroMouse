#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include "utils.h"

float distBetweenPoints(std::vector<float> p1, std::vector<float> p2){
  // function to get distance between two points
  float dist_x, dist_y;
  dist_x = p2[0] - p1[0];
  dist_y = p2[1] - p1[1];
  return (sqrt(pow(dist_x, 2) + pow(dist_y, 2)));
}

std::vector<float> lineLineIntersection(std::vector<float> l1p1, std::vector<float> l1p2, std::vector<float> l2p1, std::vector<float> l2p2)
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
        return std::vector<float> {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    }
    else
    {
        float x = (b2*c1 - b1*c2)/determinant;
        float y = (a1*c2 - a2*c1)/determinant;
        return std::vector<float> {x, y};
    }
}


bool doIntersect(std::vector<float> l1p1, std::vector<float> l1p2, std::vector<float> l2p1, std::vector<float> l2p2) {
    float d1, d2, d3, d4;
    std::vector<float> intersection;
    float distance;

    d1 = crossProduct(l2p1, l2p2, l1p1);
    d2 = crossProduct(l2p1, l2p2, l1p2);
    d3 = crossProduct(l1p1, l1p2, l2p1);
    d4 = crossProduct(l1p1, l1p2, l2p2);
    //std::cout << d1 <<  std::endl;
    //std::cout << (d1 > 0 && d2 < 0) << "   " << (d1 < 0 && d2 > 0) << "   " << (d3 > 0 && d4 < 0) << "   " << (d3 < 0 && d4 > 0) << std::endl;
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        intersection = intersectionPoint(l1p1, l1p2, l2p1, l2p2);
        std::cout << "here!!!" << std::endl;
        std::cout << intersection[0] << "   ";
        std::cout << intersection[1] << std::endl;
        distance = sqrt(pow(l1p1[0]-intersection[0], 2) + pow(l1p1[1]-intersection[2], 2));
        return distance;
    }
    return INFINITY;
}

std::vector<float> intersectionPoint(std::vector<float> l1p1, std::vector<float> l1p2, std::vector<float> l2p1, std::vector<float> l2p2) {
    std::vector<float> intersection;
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
        return std::vector<float> {NAN, NAN};
    }

    intersection[0] = (b2 * c1 - b1 * c2) / determinant;
    intersection[1] = (a1 * c2 - a2 * c1) / determinant;

    return intersection;
}

float crossProduct(std::vector<float> p, std::vector<float> q, std::vector<float> r) {
    return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
}


/*
int main() {
    LineSegment l1 = {{0, 0}, {1, 1}};
    LineSegment l2 = {{0, 1}, {1, 0}};
    Point intersection;

    if (doIntersect(l1, l2, intersection)) {
        float distance = sqrt(pow(intersection.x - l1.start.x, 2) + pow(intersection.y - l1.start.y, 2));
        std::cout << "Intersection at (" << intersection.x << ", " << intersection.y <<



std::vector<int> multiply(const std::vector<std::vector<int>>& matrix, 
                          const std::vector<int>& vector) {
  int rows = matrix.size();
  int cols = matrix[0].size();

  std::vector<int> result(rows);
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
  std::vector<std::vector<int>> matrix = {{1, 2, 3}, 
                                          {4, 5, 6}, 
                                          {7, 8, 9}};
  std::vector<int> vector = {1, 2, 3};
  std::vector<int> result = multiply(matrix, vector);

  for (const auto& item : result) {
    std::cout << item << " ";
  }

  return 0;
}
*/

