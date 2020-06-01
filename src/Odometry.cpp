#include <iostream>
#include <iostream>
#include <cmath>
#include "odometry.hpp"
#include <vector>

using namespace std;

struct position{ // class for tracking position
    float x, y, h;
    void reset(){
        x = 0;
        y = 0;
        h = 0;
    }
};

struct wayPoint{ // class for inputting waypoints
    float x, y;
};

class path{ // class for creating a path for the robot to follow
public:
    vector<wayPoint> path;
    void add(float x, float y){
        path.push_back(wayPoint{x, y});
    }
    void remove(int index){
        path.erase(path.begin() + index);
    }
    void clear(){
        path.erase(path.begin() + path.size());
    }
};

class chassis{ // class for initializing the robot chassis as a variable
    float lPrevError, rPrevError, rD, lD, lIntegral, rIntegral, kP, kI, kD;
public:
    position p;
    float width/*wheel to wheel chassis width*/,
            radius /*wheel radius*/,
            encoderMax/*encoder ticks for 1 rotation*/,
            threshold/*integral threshold for adding*/; //constants unique to each drive chassis
    bool complete;

    //sets up k constants for PID control
    void setPID_const(float P, float I, float D){
        kP = P;
        kI = I;
        kD = D;
    }

    void resetIntegrals(){
        lIntegral = 0;
        rIntegral = 0;
    }

    /*to use:
     * while(!chassisName.complete){
     *      chassisName.move =
     * }*/

    auto move(/*simple PID controller*/
            float distance/*target distance to reach*/,
            float left/*encoder values of the left chassis*/,
            float right/*encoder values of right chassis*/,
            float waitTime/*waitTime between each cycle of the while loop*/
    ){
        struct motorPower{
            float left, right;
        };
        float lError = distance - left;
        float rError = distance - right;
        if(lError == 0 && rError == 0){
            return motorPower{0, 0};
        }
        if(abs(distance - left) >= threshold){
            lIntegral = 0;
        } else{
            lIntegral++;
        }

        if(abs(distance - right) >= threshold){
            rIntegral = 0;
        } else{
            rIntegral++;
        }

        lD = kD * (lError - lPrevError) / waitTime;
        rD = kD * (rError - rPrevError) / waitTime;

        float leftPower = (lError) * kP + lIntegral * kI + lD * kD; //power allocated to left motors
        float rightPower = (rError) * kP + rIntegral * kI + rD * kD; // power allocated to right

        lPrevError = lError;
        rPrevError = rError;
        return motorPower{leftPower, rightPower};
    }
};