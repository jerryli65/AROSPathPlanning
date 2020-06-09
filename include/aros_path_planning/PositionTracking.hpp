#ifndef AROSPATHPLANNING_POSITIONTRACKING_HPP
#define AROSPATHPLANNING_POSITIONTRACKING_HPP

#endif //AROSPATHPLANNING_POSITIONTRACKING_HPP

#include "Odometry.hpp"
#include <cmath>
#include <cstdint>

#include "aros/Readable.hpp"
/**
 * PositionTracking.hpp and PositionTracking.cpp are files containing the functions
 * dealing with the tracking of Robot position using encoder wheels
 */

using namespace aros::ChassisInit;

namespace aros::PositionTracking{
    typedef int64_t EncoderType; ///change datatype EncoderType depending on what values the encoders output

    template<typename T>
    struct Position{
        /**
         * x: x position
         * y: y position
         * h: heading (or angle in relative to starting position)
         */
        T x, y, angle;
    };
    /**
     * stores the intended reset position
     */
    struct ResetState{
        Position<float> position;
        EncoderType right_encoder;
        EncoderType left_encoder;
        EncoderType back_encoder;
    };

    class PositionTracker: Readable<Position<float>>{
        Position<float> _last_position;
        const ChassisDefinition _chassis_definition;
        EncoderType _last_left_position, _last_right_position, _last_back_position;
        ResetState _reset_state;

    public:
        PositionTracker(const ResetState& reset_state, const ChassisDefinition& chassis_definition);
        /**
         * @param right : right tracking wheel encoder values
         * @param left : left tracking wheel encoder values
         * @param back : back tracking wheel encoder values
         * @return returns the updated position of the robot
         */
        auto track(EncoderType right, EncoderType left, EncoderType back) -> Position<float>;
        /**
         * @param reset_state
         * @return
         */
        auto reset(const ResetState& reset_state);

        //Readable
    private:
        auto value() -> Position<float> override;
    };
}
