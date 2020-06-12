#ifndef AROSPATHPLANNING_ODOMETRY_HPP
#define AROSPATHPLANNING_ODOMETRY_HPP
#endif //AROSPATHPLANNING_ODOMETRY_HPP
#include <vector>
#include <cstdint>

namespace aros::ChassisInit{
    /// class for initializing the robot chassis as a variable
    class ChassisDefinition{
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
        ///Returns Back wheel to turn center value
        [[nodiscard]] auto Back() const -> float;
        ///Returns Right wheel to turn center value
        [[nodiscard]] auto Right() const -> float;
        ///Returns Left wheel to turn center value
        [[nodiscard]] auto Left() const -> float;
        ///Returns Wheel diameter Value
        [[nodiscard]] auto diameter() const -> float;
        ///Returns the number of ticks per revolution on the used encoder
        [[nodiscard]] auto ticksPerRev() const -> float;
    };
}


