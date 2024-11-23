#pragma once
#ifndef turnPID_H
#define turnPID_H

#include "cmath"
#include "vex.h"
#include "motors.h"
#include <iostream>

class turnPID
{
private:
    double position;
    double error;
    double i; // integral
    double d;
    int target;
    double kp = 0.5;
    double ki = 0;
    double kd = 0.1;
    double drive;
    void runTurnPID();
    double prev;

public:
    inline turnPID() {
    }

    /*void reset(vex::motor_group Left, vex::motor_group Right)
    {
        error = 0;
        prev = 0;
        i = 0;
        Left.resetPosition();
        Right.resetPosition();
    }*/

    

    inline void tpUpdate()
    {
        position = InertialSensor.heading();


        error = target - position;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        
        std::cout<<abs(position - target)<<std::endl;
        // std::cout<<error<<std::endl;

        Brain.Screen.clearLine();
        Brain.Screen.print(error);

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

    inline void runTurnPID(double targetVal/*, vex::motor_group Left, vex::motor_group Right*/)
    {
        int time = 0;
        // std::cout<<position<<std::endl;
        position = 0;
        //InertialSensor.resetHeading();
        target = targetVal;
        // std::cout<<position<<std::endl;
        // wait(1, vex::sec);
        while (abs(position - target) > 2) {
            tpUpdate();
            // std::cout<<"h"<<std::endl;
            // std::cout<<position<<std::endl;
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, (kp * error) + (ki * i) - (kd * d), vex::pct);
            Right.spin(vex::reverse, (kp * error) + (ki * i)  - (kd * d), vex::pct);
            // tpUpdate();

            if (position > 360) {
                position -= 360;
            } else if (position < -360) {
                position += 360;
            }
            
            time += 20;
            vex::wait(20, vex::msec);
            if (time >= 2000) {
                break;
            }
        } 
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        std::cout<<"done :)"<<std::endl;

        vex::wait(20, vex::msec);
        
    }
};

#endif