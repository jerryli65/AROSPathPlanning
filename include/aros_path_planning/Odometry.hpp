#ifndef AROSPATHPLANNING_ODOMETRY_HPP
#define AROSPATHPLANNING_ODOMETRY_HPP

#include <vector>
#include <cstdint>

namespace aros::path_panning{
    typedef int64_t EncoderType;

    struct Position{ // class for tracking position
        float x, y, h;

        /**
         * This function resets the position
         */
        auto reset() -> void;
    };

    struct WayPoint{ // class for inputting waypoints
        float x, y;
    };

    class Path{ // class for creating a path for the robot to follow
        std::vector<WayPoint> path;

    public:
        Path();
        explicit Path(std::vector<WayPoint> path);
        explicit Path(std::vector<WayPoint>&& path);

        void add(float x, float y);
        void remove(int index);
        void clear();
    };

    /**
     * Chassis definition
     */
    class ChassisDefinition{ // class for initializing the robot chassis as a variable
        float lPrevError, rPrevError, rD, lD, lIntegral, rIntegral, kP, kI, kD;
        Position p;
        //constants unique to each drive chassis
        ///wheel to wheel chassis width
        float _width;
        ///wheel radius
        float radius;
        ///encoder ticks for 1 rotation
        float encoderMax;
        ///integral threshold for adding
        float threshold;
        bool complete;

    public:
        /**
         *
         * @param l_prev_error
         * @param r_prev_error
         * @param r_d
         * @param l_d
         * @param l_integral
         * @param r_integral
         * @param k_p
         * @param k_i
         * @param k_d
         * @param p
         * @param width
         * @param radius
         * @param encoder_max
         * @param threshold
         * @param complete
         */
        ChassisDefinition(float l_prev_error, float r_prev_error, float r_d, float l_d, float l_integral, float r_integral, float k_p, float k_i, float k_d, const Position& p, float width, float radius, float encoder_max, float threshold,
                          bool complete);


        [[nodiscard]] auto width() const -> float;

        //sets up k constants for PID control
        void setPID_const(float P, float I, float D);

        void resetIntegrals();

        /*to use:
         * while(!chassisName.complete){
         *      chassisName.move =
         * }*/

        /**
         * simple PID controller
         * @param distance target distance to reach
         * @param left encoder values of the left chassis
         * @param right encoder values of right chassis
         * @param waitTime wait time between each cycle of the while loop
         * @return
         */
        auto move(float distance, float left, float right, float waitTime);
    };

    struct WheelPower{
        float right_side, left_side;
    };

    class PathControl{
        ChassisDefinition _definition;

    public:
        PathControl();
        PathControl(ChassisDefinition definition);

        auto calculate_values(const Path& path, const int32_t& left_encoder, const int32_t& right_encoder) -> WheelPower;
    };
}

#endif //AROSPATHPLANNING_ODOMETRY_HPP
