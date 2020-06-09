#ifndef AROSPATHPLANNING_ODOMETRY_HPP
#define AROSPATHPLANNING_ODOMETRY_HPP
#endif //AROSPATHPLANNING_ODOMETRY_HPP
#include <vector>
#include <cstdint>

namespace aros::ChassisInit{
    /**
     * Chassis definition
     */
    class ChassisDefinition{ /// class for initializing the robot chassis as a variable
        ///constants unique to each drive chassis
        ///distances from turn center to wheel center for both wheels
        float _sR;
        float _sL;
        float _sB;
        ///encoder wheel radius
        float _radius;
        ///encoder ticks for 1 rotation
        float _encoderMax;
    public:
        [[nodiscard]] auto sB() const -> float;
        [[nodiscard]] auto sR() const -> float;
        [[nodiscard]] auto sL() const -> float;
        [[nodiscard]] auto radius() const -> float;
        [[nodiscard]] auto encoderMax() const -> float;
    };
}


