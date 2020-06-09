#include "aros_path_planning/PositionTracking.hpp"

namespace aros::PositionTracking{
    auto _Position::Track(EncoderType right, EncoderType left, EncoderType back){
        float dx;
        float dy;
        float distanceLeft = ((left - prevL) / c.ticksPerRev()) * M_PI * c.diameter();
        float distanceRight = ((right - prevR) / c.ticksPerRev()) * M_PI * c.diameter();
        float distanceBack = ((back - prevB) / c.ticksPerRev()) * M_PI * c.diameter();
        float dTheta = (distanceLeft - distanceRight) / (c.Left() + c.Right());
        if(dTheta == 0){
            dx = distanceBack * cos(p.h) + distanceRight * sin(p.h); ///This works
            dy = distanceRight * cos(p.h) + distanceBack * sin(p.h);
        } else{
            dx = (2 * sin(M_PI / 2) * ((distanceBack / dTheta) + c.Back())) * cos(p.h) + (2 * sin(M_PI / 2) * ((distanceRight / dTheta) + c.Right())) * sin(p.h); ///Idk if this does though
            dy = (2 * sin(M_PI / 2) * ((distanceRight / dTheta) + c.Right())) * cos(p.h) + (2 * sin(M_PI / 2) * ((distanceBack / dTheta) + c.Back())) * sin(p.h);
        }
        p.y += dy;
        p.x += dx;
        p.h += dTheta;
    };
}