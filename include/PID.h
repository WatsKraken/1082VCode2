#pragma once
#include "cmath"
#include "vex.h"
#include "motors.h"

class PID
{
private:
    double _position;
    double error;
    double i; // integral
    double d;
    int target;
    double kp = 0.5;
    double ki = 0;
    double kd = 0.1;
    double drive;
    void runPID();
    double prev;

public:
    PID() {
    }

    void reset()
    {
        error = 0;
        prev = 0;
        i = 0;
        Left.resetPosition();
        Right.resetPosition();
    }

    void update()
    {
        _position = ((fabs(Ml.position(vex::turns)) + fabs(Mr.position(vex::turns))) / 2.0) * M_PI * 3.25;

 
        error = target - _position;

        

        i = i + error + (prev - error) / 2.0;
        d = error - prev;
        prev = error;
        if (error == 0)
        {
            i = 0;
        }

        if (fabs(i) >= 100)
        {
            i = (i / fabs(i)) * 100;
        }
    }

    void runPID(double targetVal, double timeLimit)
    {
        _position = 0;

        int time = 0;
        target = targetVal;
        while (fabs(_position - target) > 0.2) {
            update();
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);
            Right.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);

            time += 20;
            vex::wait(20, vex::msec);
            if (time >= timeLimit * 1000) {
                break;
                Brain.Screen.print("time up");
            }
        }

        // vex::wait(20, vex::msec);
    }
};
