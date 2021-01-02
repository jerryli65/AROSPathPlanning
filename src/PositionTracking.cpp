#include "aros_path_planning/PositionTracking.hpp"
// TODO: How many wheels does each robot have? What is the language standard, 11 or 17? docs.idew.org code robotics encoders
//NOTE: Do not auto-format, this formatting doesn't like that

namespace aros::PositionTracking{ // namespace containing functions to track robot position


    template<typename EncoderType, typename T>
    auto PositionTracker<EncoderType, T>::track(EncoderType right, EncoderType left, EncoderType back) -> Position<T>{
        /* Step 1: Store encoders in local variables (left, right, and back) */

        /* Step 2: Calculate the change in encoder values since previous cycle,
         * convert it to wheel rotations, and then how far the wheel has travelled
         * and store them in variables (delta_left, delta_right, and delta_back) */
        float delta_left = ((left - _last_left_position) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();
        float delta_right = ((right - _last_right_position) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();
        float delta_back = ((back - _last_back_position) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();

        /* Step 3: Update the total change in the values of the left and right encoders
         *  Assign these to variables as well (delta_left_absolute, delta_right_absolute) */
        float delta_left_absolute = ((left - _reset_state.left_encoder) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();
        float delta_right_absolute = ((right - _reset_state.right_encoder) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();

        /* Step 4: Calculate new absolute orientation */
        float theta_current = _reset_state.position.angle + (delta_left_absolute - delta_right_absolute) / (_chassis_definition.Left() + _chassis_definition.Right());

        /* Step 5: Calculate change in angle */
        float delta_theta = theta_current - _last_position.angle;

        float delta_x_local;
        float delta_y_local;

        /* Step 6: If the change in angle is 0, local offset is just (x: back wheel, y: average between left and right wheels)
         *  Otherwise, calculate offset based off of back, left, and right wheels and the change in angle */
        // TODO: UNDERSTAND THIS CODE
        if(delta_theta == 0){
            delta_x_local = delta_back;
            delta_y_local = (delta_right + delta_left) / 2; // NOTE: just delta_right or delta_left
        } else{
            delta_x_local = 2 * sin(delta_theta) * (delta_back / delta_theta + _chassis_definition.Back());
            float y_r = 2 * sin(delta_theta) * (delta_right / delta_theta + _chassis_definition.Right());
            float y_l = 2 * sin(delta_theta) * (delta_left / delta_theta - _chassis_definition.Left());
            delta_y_local = (y_r + y_l) / 2;
        }
        /* Step 7: Calculate average orientation */
        float theta_average = _reset_state.position.angle + delta_theta / 2;  //NOTE: should reset_state.position.angle be _last_position.angle ?

        float delta_x, delta_y;
        /* Step 8: Calculate global offset by "rotating" local offset by the average orientation */
        Rotate(delta_x_local, delta_y_local, -theta_average, delta_x, delta_y); //NOTE: What does theta_average do?

        /* Step 9: Calculate new absolute position */
        _last_position.x += delta_x;
        _last_position.y += delta_y;
        _last_position.angle = theta_current;

        /* Step 10: Update "previous encoder" variables to hold current encoder values */
        _last_left_position = left;
        _last_right_position = right;
        _last_back_position = back;

        //Returns new position of robot
        return _last_position;
    }
    template<typename EncoderType, typename T>
    PositionTracker<EncoderType, T>::PositionTracker(const ResetState<EncoderType, T>& reset_state, const ChassisDefinition& chassis_definition) : _reset_state(reset_state), _last_position(reset_state.position),
                                                                                                                                                   _chassis_definition(chassis_definition),
                                                                                                                                                   _last_left_position(reset_state.left_encoder),
                                                                                                                                                   _last_right_position(reset_state.right_encoder),
                                                                                                                                                   _last_back_position(reset_state.back_encoder){}

    template<typename EncoderType, typename T>
    auto PositionTracker<EncoderType, T>::reset(const ResetState<EncoderType, T>& reset_state) -> void{
        _reset_state = reset_state;
        _last_position = reset_state.position;
        _last_left_position = reset_state.left_encoder;
        _last_right_position = reset_state.right_encoder;
        _last_back_position = reset_state.back_encoder;
    }

    template<typename EncoderType, typename T>
    auto PositionTracker<EncoderType, T>::value() -> Position<float>{
        return _last_position;
    }
}
