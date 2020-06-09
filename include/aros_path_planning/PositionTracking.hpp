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
        float x, y, angle;
    };

    struct ResetState{
        Position position;
        EncoderType right_encoder;
        EncoderType left_encoder;
        EncoderType back_encoder;
    };

    class PositionTracker{
        Position _last_position;
        const ChassisDefinition _chassis_definition;
        EncoderType _last_left_position, _last_right_position, _last_back_position;
        ResetState _reset_state;
    public:
        PositionTracker(const ResetState& reset_state, const ChassisDefinition& chassis_definition);
        /**
         *
         * @param right right tracking wheel encoder values
         * @param left left tracking wheel encoder values
         * @param back back tracking wheel encoder values
         * @return returns the motor power values of the left and right chassis
         */
        auto track(EncoderType right, EncoderType left, EncoderType back) -> Position;
        auto reset(const ResetState& reset_state);
    };
}
