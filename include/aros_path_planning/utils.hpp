//
// Created by david on 6/10/2020.
//
#include <cmath>

#ifndef AROSPATHPLANNING_UTILS_HPP
#define AROSPATHPLANNING_UTILS_HPP

#endif //AROSPATHPLANNING_UTILS_HPP
namespace aros::utils{
    template<typename T>
    inline auto CartesianToPolar(T x, T y, T& r_out, T& theta_out);

    template<typename T>
    inline auto PolarToCartesian(T r, T theta, T& x_out, T& y_out);
}