#include "aros_path_planning/PathPlanning.hpp"

namespace aros::PathPlanning{
    void Path::add(WayPoint d){
        c.push_back(d);
    }
}
