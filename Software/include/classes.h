#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <cmath>

class Robot {
    public:
        std::vector<float> position;
        std::vector<float> direction;
        float speedForward;
        float radius;

        Robot(float x, float y, float x_dir, float y_dir, float radius_);
        void updatePosition(float SpeedL, float SpeedR);
};

#endif
