#include "aros_path_planning/Odometry.hpp"

#include <cmath>
#include <utility>

namespace aros::path_panning{
    auto Position::reset() -> void{
        x = 0;
        y = 0;
        h = 0;
    }

    void Path::add(float x, float y){
        path.push_back(WayPoint{x, y});
    }
    void Path::remove(int index){
        path.erase(path.begin() + index);
    }
    void Path::clear(){
        path.erase(path.begin() + path.size());
    }
    Path::Path(std::vector<WayPoint> path) : path(std::move(path)){

    }
    Path::Path(std::vector<WayPoint>&& path) : path(std::move(path)){

    }
    Path::Path() : path(){

    }
    void ChassisDefinition::setPID_const(float P, float I, float D){

        kP = P;
        kI = I;
        kD = D;

    }
    void ChassisDefinition::resetIntegrals(){

        lIntegral = 0;
        rIntegral = 0;

    }
    auto ChassisDefinition::move(float distance, float left, float right, float waitTime){

        struct motorPower{
            float left, right;
        };
        float lError = distance - left;
        float rError = distance - right;
        if(lError == 0 && rError == 0){
            return motorPower{0, 0};
        }
        if(std::abs(distance - left) >= threshold){
            lIntegral = 0;
        } else{
            lIntegral++;
        }

        if(std::abs(distance - right) >= threshold){
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
    ChassisDefinition::ChassisDefinition(float l_prev_error, float r_prev_error, float r_d, float l_d, float l_integral, float r_integral, float k_p, float k_i, float k_d, const Position& p, float width, float radius, float encoder_max,
                                         float threshold, bool complete) : lPrevError(l_prev_error), rPrevError(r_prev_error), rD(r_d), lD(l_d), lIntegral(l_integral), rIntegral(r_integral), kP(k_p), kI(k_i), kD(k_d), p(p), _width(width),
                                                                           radius(radius), encoderMax(encoder_max), threshold(threshold), complete(complete){}
    auto ChassisDefinition::width() const -> float{
        return _width;
    }
}
