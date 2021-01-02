#ifndef AROSPATHPLANNING_PATHPLANNING_HPP
#define AROSPATHPLANNING_PATHPLANNING_HPP
#include "aros_path_planning/PositionTracking.hpp"

namespace aros::PathPlanning{
    class Path{
        struct WayPoint{ /// class for inputting waypoints
            float x, y;
        };
        std::vector<WayPoint> c;
    public:
        void add(WayPoint d);
    };

    class PathControl{
        ChassisDefinition _definition;
    };
}

#endif //AROSPATHPLANNING_PATHPLANNING_HPP