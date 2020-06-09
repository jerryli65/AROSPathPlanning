#ifndef AROSPATHPLANNING_POSITIONTRACKING_HPP
#define AROSPATHPLANNING_POSITIONTRACKING_HPP

#endif //AROSPATHPLANNING_POSITIONTRACKING_HPP

#include "ChassisDefinition.hpp"
#include <cmath>
#include <cstdint>
#include "utils.hpp"
#include "aros/Readable.hpp"
/**
 * PositionTracking.hpp and PositionTracking.cpp are files containing the functions
 * dealing with the tracking of Robot position using encoder wheels
 */

using namespace aros::ChassisInit;

namespace aros::PositionTracking{
    //change datatype EncoderType depending on what values the encoders output
    typedef int32_t EncoderType;

    template<typename T>
    /** struct intended to store a position*/
    struct Position{
        /** x: x position
         * y: y position
         * h: heading (or angle in relative to starting position) */
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

    ///Readable position values
    class PositionTracker: Readable<Position<float>>{
        Position<float> _last_position;
        const ChassisDefinition _chassis_definition;
        EncoderType _last_left_position, _last_right_position, _last_back_position;
        ResetState _reset_state;

    public:
        PositionTracker(const ResetState& reset_state, const ChassisDefinition& chassis_definition);
        /** Tracks the x-y position of the robot based on encoder wheel values
         * @param right : right tracking wheel encoder values
         * @param left : left tracking wheel encoder values
         * @param back : back tracking wheel encoder values
         * @return returns the updated position of the robot
         */
        auto track(EncoderType right, EncoderType left, EncoderType back) -> Position<float>;
        /** Resets the wheel encoder values to a "reset state" stored beforehand
         * @param reset_state position to reset encoder values
         */
        auto reset(const ResetState& reset_state) -> void;

        //Readable
    private:
        auto value() -> Position<float> override;
    };
}
