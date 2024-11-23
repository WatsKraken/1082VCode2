#pragma once
#include "cmath"
#include "vex.h"
#include "motors.h"

public:
    double position;
    double error;
    double i; // integral
    double d;
    int target;
    double kp = 0.5;
    double ki = 0.2;
    double kd = 0.1;
    double drive;
    double prev;
    void reset();
    int time; 
    bool errorChanging = true;
