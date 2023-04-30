#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
  public:
    PIDController(float kp, float ki, float kd, float maxOutput, float minOutput);
    float calculate(float setPoint, float processVariable, float deltaTime);
    
  private:
    float kp;
    float ki;
    float kd;
    float maxOutput;
    float minOutput;
    float lastError;
    float totalError;
};

#endif