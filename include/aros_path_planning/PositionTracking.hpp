#ifndef AROSPATHPLANNING_POSITIONTRACKING_HPP
#define AROSPATHPLANNING_POSITIONTRACKING_HPP

#endif //AROSPATHPLANNING_POSITIONTRACKING_HPP

#include "Odometry.hpp"
#include <cmath>
#include <cstdint>

using namespace aros::ChassisInit;

namespace aros::PositionTracking{
    typedef int64_t EncoderType;
    struct Position{
        /**
         * x: x position
         * y: y position
         * h: heading (or angle in relative to starting position)
         */
        float x, y, h;
    };

    class _Position{
        Position p;
        ChassisDefinition c;
        EncoderType prevL, prevR, prevB;
    public:
        /**
         *
         * @param right : right tracking wheel encoder values
         * @param left : left tracking wheel encoder values
         * @return returns the motor power values of the left and right chassis
         */
        auto Track(EncoderType right, EncoderType left, EncoderType back);
    };
}