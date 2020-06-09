#include "aros_path_planning/Odometry.hpp"

namespace aros::ChassisInit{
    auto ChassisDefinition::diameter() const -> float{
        return _diameter;
    }
    auto ChassisDefinition::ticksPerRev() const -> float{
        return _ticksPerRev;
    }
    auto ChassisDefinition::Back() const -> float{
        return _Back;
    }
    auto ChassisDefinition::Right() const -> float{
        return _Right;
    }
    auto ChassisDefinition::Left() const -> float{
        return _Left;
    }
}
