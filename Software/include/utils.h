#include <vector>

#ifndef UTILS_H
#define UTILS_H

float distBetweenPoints(std::vector<float> p1, std::vector<float> p2);
std::vector<float> lineLineIntersection(std::vector<float> l1p1, std::vector<float> l1p2, std::vector<float> l2p1, std::vector<float> l2p2);
bool doIntersect(std::vector<float> l1p1, std::vector<float> l1p2, std::vector<float> l2p1, std::vector<float> l2p2);
std::vector<float> intersectionPoint(std::vector<float> l1p1, std::vector<float> l1p2, std::vector<float> l2p1, std::vector<float> l2p2);
float crossProduct(std::vector<float> p, std::vector<float> q, std::vector<float> r);

#endif
