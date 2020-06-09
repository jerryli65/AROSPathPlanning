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
        float _Right;
        float _Left;
        float _Back;
        ///encoder wheel radius
        float _diameter;
        ///encoder ticks for 1 rotation
        float _ticksPerRev;
    public:
        [[nodiscard]] auto Back() const -> float;
        [[nodiscard]] auto Right() const -> float;
        [[nodiscard]] auto Left() const -> float;
        [[nodiscard]] auto diameter() const -> float;
        [[nodiscard]] auto ticksPerRev() const -> float;
    };
}


