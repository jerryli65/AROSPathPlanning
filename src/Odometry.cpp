#include "aros_path_planning/Odometry.hpp"
#include <utility>

namespace aros::ChassisInit{
    auto ChassisDefinition::radius() const -> float{
        return _radius;
    }
    auto ChassisDefinition::encoderMax() const -> float{
        return _encoderMax;
    }
    auto ChassisDefinition::sB() const -> float{
        return _sB;
    }
    auto ChassisDefinition::sR() const -> float{
        return _sR;
    }
    auto ChassisDefinition::sL() const -> float{
        return _sL;
    }
}
