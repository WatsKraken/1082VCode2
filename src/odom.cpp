#include "vex.h"
#include "motors.h"
using namespace std;

class odom {
    private:
    //std::vector<int> _positionVector = {1, 2, 3, 4};
    double tL /*Distance from the tracking center to the left tracking wheel*/= 4.625;
    double tR /*Distance from the tracking center to the right tracking wheel*/= 4.625;
    //double tR /*Distance from the tracking center to the right tracking wheel*/= 4.625;
    double L /*The distance that the left-side tracking wheel has traveled*/;
    double R /*The distance that the right-side tracking wheel has traveled*/;
    double LPrev = 0;
    double RPrev = 0;
    double Δθ = 0;
    double Δx = 0;
    double Δx_local = 0;
    double Δy = 0;
    double Δy_local = 0;
    double x = 0;
    double y = 0;
    double θ = 0;
    double r = 1;
    double _arcLength = 0;


    public:
    odom(double initialRotation = 0){}

    void update() {
        Δx = 0;
        Δy = 0;
        Δθ = InertialSensor.heading() - θ;
        //  L = Ml.
        //temporarily using left motors instead of a left tracking wheel and vice versa for the right
        L = Left.position(vex::deg) * M_PI * 3.25 - LPrev;
        R = Right.position(vex::deg) * M_PI * 3.25 - RPrev;
        _arcLength = (L + R)/2;
        r = _arcLength/Δθ;
        Δx_local = r*(1-cos(Δθ));
        Δy_local = r*sin(Δθ);
        Δx = (Δx_local * cos(θ)) + (Δy_local * sin(θ));
        Δy = (Δx_local * sin(θ)) + (Δy_local * cos(θ));

        θ = InertialSensor.heading();
        x += Δx;
        y += Δy;

        LPrev = Left.position(vex::deg) * M_PI * 3.25;
        RPrev = Left.position(vex::deg) * M_PI * 3.25;
        vex::wait(10, vex::msec);
    }
};