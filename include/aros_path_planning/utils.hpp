#include <cmath>
#ifndef AROSPATHPLANNING_UTILS_HPP
#define AROSPATHPLANNING_UTILS_HPP
namespace aros::utils{
    /** Converts a Cartesian coordinate to a polar coordinate
    *
    * @param x input cartesian x-coordinate
    * @param y input cartesian y-coordinate
    * @param r_out output polar r-coordinate
    * @param theta_out output polar theta coordinate
    * @return a polar coordinate, given an input Cartesian coordinate
    */


    template<typename T>
    inline auto CartesianToPolar(T x, T y, T& r_out, T& theta_out) -> void{
        r_out = sqrt(x * x + y * y);
        theta_out = atan(y / x);
    }
    /** Converts a polar coordinate to a cartesian coordinate
     *
     * @param r input Polar r-coordinate
     * @param theta input Polar theta coordinate
     * @param x_out output Cartesian x-coordinate
     * @param y_out output Cartesian y-coordinate
     * @return a Cartesian coordinate, given an input Polar coordinate
     */


    template<typename T>
    inline auto PolarToCartesian(T r, T theta, T& x_out, T& y_out) -> void{
        x_out = r * cos(theta);
        y_out = r * sin(theta);
    }

    /**
    *
    * @param x input previous global y-coordinate
    * @param y input previous global x-coordiante
    * @param rotate_theta input change in theta to "rotate" the local offset to calculate global offset (Track - step 8)
    * @param x_out outputs new global x-coordinate
    * @param y_out outputs new global y-coordinate
    * @return updates global coordinates
    */
    template<typename T>
    inline auto Rotate(T x, T y, T rotate_theta, T& x_out, T& y_out){
        T r, theta;
        CartesianToPolar(x, y, r, theta);
        theta += rotate_theta;
        PolarToCartesian(r, theta, x_out, y_out);
    }
}
#endif //AROSPATHPLANNING_UTILS_HPP
