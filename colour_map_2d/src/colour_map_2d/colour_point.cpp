#include "colour_map_2d/colour_point.h"

namespace ColourMap2D {
////CONSTRUCTORS
ColourPoint::ColourPoint(double x_in, double y_in, uint8_t r_in, uint8_t g_in, uint8_t b_in)
    :x_(x_in), y_(y_in), r_(r_in), g_(g_in), b_(b_in)
    {}

ColourPoint::ColourPoint(ColourPoint point, double x_noise, double y_noise)
    :r_(point.getR()), g_(point.getG()), b_(point.getB())
{
    x_ = point.getX() + x_noise;
    y_ = point.getY() + y_noise;
}

////GETTERS
double ColourPoint::getX() const { return x_; }
double ColourPoint::getY() const { return y_; }
uint8_t ColourPoint::getR() const { return r_; }
uint8_t ColourPoint::getG() const { return g_; }
uint8_t ColourPoint::getB() const { return b_; }
}
