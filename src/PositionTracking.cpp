#include "aros_path_planning/PositionTracking.hpp"

namespace aros::PositionTracking{
    template<typename T>
    inline auto CartesianToPolar(T x, T y, T& r_out, T& theta_out){
        r_out = sqrt(x * x + y * y);
        theta_out = atan(y / x);
    }

    template<typename T>
    inline auto PolarToCartesian(T r, T theta, T& x_out, T& y_out){
        x_out = r * cos(theta);
        y_out = r * sin(theta);
    }

    template<typename T>
    inline auto Rotate(T x, T y, T rotate_theta, T& x_out, T& y_out){
        float r, theta;
        CartesianToPolar(x, y, r, theta);
        theta += rotate_theta;
        PolarToCartesian(r, theta, x_out, y_out);
    }

    auto PositionTracker::track(EncoderType right, EncoderType left, EncoderType back) -> Position<float>{
        float delta_left = ((left - _last_left_position) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();
        float delta_right = ((right - _last_right_position) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();
        float delta_back = ((back - _last_back_position) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();

        float delta_left_absolute = ((left - _reset_state.left_encoder) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();
        float delta_right_absolute = ((right - _reset_state.right_encoder) / _chassis_definition.ticksPerRev()) * M_PI * _chassis_definition.diameter();

        float theta_current = _reset_state.position.angle + (delta_left_absolute - delta_right_absolute) / (_chassis_definition.Left() + _chassis_definition.Right());
        float delta_theta = theta_current - _last_position.angle;

        float delta_x_local;
        float delta_y_local;
        if(delta_theta == 0){
            delta_x_local = delta_back; ///This works
            delta_y_local = (delta_right + delta_left) / 2;
        } else{
            delta_x_local = 2 * sin(delta_theta) * (delta_back / delta_theta + _chassis_definition.Back());
            float y_r = 2 * sin(delta_theta) * (delta_right / delta_theta + _chassis_definition.Right());
            float y_l = 2 * sin(delta_theta) * (delta_left / delta_theta - _chassis_definition.Left());
            delta_y_local = (y_r + y_l) / 2;
        }

        float theta_average = _reset_state.position.angle + delta_theta / 2;

        float delta_x, delta_y;
        Rotate(delta_x_local, delta_y_local, -theta_average, delta_x, delta_y);

        _last_position.x += delta_x;
        _last_position.y += delta_y;
        _last_position.angle = theta_current;

        _last_left_position = left;
        _last_right_position = right;
        _last_back_position = back;

        return _last_position;
    }

    PositionTracker::PositionTracker(const ResetState& reset_state, const ChassisDefinition& chassis_definition) : _reset_state(reset_state), _last_position(reset_state.position), _chassis_definition(chassis_definition),
                                                                                                                   _last_left_position(reset_state.left_encoder), _last_right_position(reset_state.right_encoder),
                                                                                                                   _last_back_position(reset_state.back_encoder){}
    auto PositionTracker::reset(const ResetState& reset_state){
        _reset_state = reset_state;
        _last_position = reset_state.position;
        _last_left_position = reset_state.left_encoder;
        _last_right_position = reset_state.right_encoder;
        _last_back_position = reset_state.back_encoder;
    }
    auto PositionTracker::value() -> Position<float>{
        return _last_position;
    }
}
