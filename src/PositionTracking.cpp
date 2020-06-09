#include "aros_path_planning/PositionTracking.hpp"

namespace aros::PositionTracking{
    auto _Position::Track(EncoderType right, EncoderType left, EncoderType back){

        float dl = 2 * ((left - prevL) / c.encoderMax()) * M_PI * c.radius();
        float dr = 2 * ((right - prevR) / c.encoderMax()) * M_PI * c.radius();
        float db = 2 * ((back - prevB) / c.encoderMax()) * M_PI * c.radius();
        float dTheta = (dl - dr) / (c.sL() + c.sR());


        float dx = 2 * sin(M_PI / 2) * ((db / dTheta) + c.sB());
        float dy = 2 * sin(M_PI / 2) * ((dr / dTheta) + c.sR());

        p.y += dy;
        p.x += dx;
        p.h += dTheta;
    };
}