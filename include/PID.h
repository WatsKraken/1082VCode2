#pragma once
#include "cmath"
#include "vex.h"
#include "motors.h"

namespace _PID {
    double position;
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
    int _time = 0;
    bool errorChanging = true;
}

using namespace _PID;
class PID
{
    

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
        position = ((abs(Tl.position(vex::turns)) + abs(Tr.position(vex::turns))) / 2.0) * M_PI * 3.25;
        error = target - position;

        if (error = prev) {
            printToConsole("The error is not changing. PID stopping.");
            Brain.Screen.print("The error is not changing. PID stopping.");
            errorChanging = false;
        } 

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

    bool isStopped() {
        if ((Left.velocity(vex::rpm) + Right.velocity(vex::rpm))/2 <= 1 || ((error == prev) && (Left.velocity(vex::rpm) + Right.velocity(vex::rpm))/2 <= 1)) return true;
        else return false;
    }

    void runPID(double targetVal, double timeLimit)
    {
        reset();
        target = targetVal;
        while (abs(position - target) > 0.2 && errorChanging) {
            update();
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);
            Right.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);

            if (isStopped()) { break; }
            _time += 20;
            vex::wait(20, vex::msec);
            if (_time >= timeLimit * 1000) {
                break;
            }
        }

        vex::wait(20, vex::msec);
    }
};
