#include <stdint.h>

#ifndef COLOUR_POINT_H
#define COLOUR_POINT_H

namespace ColourMap2D{

class ColourPoint{
    double x_;
    double y_;
    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
public:
    ////CONSTRUCTORS
    ColourPoint(double x_in, double y_in, uint8_t r_in, uint8_t g_in, uint8_t b_in);
    ColourPoint(ColourPoint point, double x_noise, double y_noise);
    
    ////GETTERS
    double getX() const;
    double getY() const;
    uint8_t getR() const;
    uint8_t getG() const;
    uint8_t getB() const;
};

}

#endif // COLOUR_POINT_H
