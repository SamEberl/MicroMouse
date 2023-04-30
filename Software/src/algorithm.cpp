#include <cmath>
#include <vector>
#include <stdio.h>
#include <iostream>
#include "defs.h"
#include "estimations.h"
#include "algorithm.h"

using namespace std;


// // Define PID constants
// float Kp = 1.0;
// float Ki = 0.5;
// float Kd = 0.2;

// float PID(RobotEst mouse) {
//     // Define variables
//     float setpoint = 10.0;
//     float error = 0.0;
//     float last_error = 0.0;
//     float integral = 0.0;
//     float derivative = 0.0;
//     float output = 0.0;
//     float input = 0.0;

//     // Read input value
//     input = mouse.rob_dir;

//     // Calculate error
//     error = setpoint - input;

//     // Calculate integral
//     integral += error;

//     // Calculate derivative
//     derivative = error - last_error;

//     // Calculate output
//     output = Kp * error + Ki * integral + Kd * derivative;

//     // Update last error
//     last_error = error;

//     // Write output value
//     return output;
// }

// // Function to read input value
// float readInput() {
//     // Code to read input value
//     return 0.0;
// }

// // Function to write output value
// void writeOutput(float output) {
//     // Code to write output value
//     std::cout << "Output: " << output << std::endl;
// }


PIDController::PIDController(float kp, float ki, float kd, float maxOutput, float minOutput) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
    lastError = 0;
    totalError = 0;
}

float PIDController::calculate(float setPoint, float processVariable, float deltaTime) {
    float error = setPoint - processVariable;
    float derivative = (error - lastError) / deltaTime;
    totalError += error * deltaTime;
    float output = kp * error + ki * totalError + kd * derivative;
    output = min(output, maxOutput);
    output = max(output, minOutput);
    lastError = error;
    return output;
}
    
void turn(RobotEst mouse, float target_dir){

}

void drive_to_next_cell(vector <float> start, vector <float> end){
    
}


