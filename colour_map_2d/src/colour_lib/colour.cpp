#include "colour_lib/colour.h"

namespace ColourLib {

ColourInfo::ColourInfo(Colour colour_input, std::string name_input,
                       std::vector<uint8_t> h_input, std::vector<uint8_t> s_input, std::vector<uint8_t> v_input,
                       uint8_t r_input, uint8_t g_input, uint8_t b_input)
    :colour(colour_input), name(name_input), h(h_input), s(s_input), v(v_input), r(r_input), g(g_input), b(b_input)
{}

ColourInfo::ColourInfo(Colour colour_input, std::__cxx11::string name_input,
                       uint8_t r_input, uint8_t g_input, uint8_t b_input)
    :colour(colour_input), name(name_input), r(r_input), g(g_input), b(b_input)
{}


Colour Identifier::identifyHSVThresh(cv::Vec3b RGB)
{
    cv::Vec3b hsv = Identifier::vecRGB2HSV(RGB);
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(Identifier::inThresh(COLOURS.at(i).h, hsv[0]) &&
           Identifier::inThresh(COLOURS.at(i).s, hsv[1]) &&
           Identifier::inThresh(COLOURS.at(i).v, hsv[2])) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

Colour Identifier::identifyHSVThresh(uint8_t r, uint8_t g, uint8_t b)
{
    cv::Vec3b RGB(r, g, b);
    cv::Vec3b hsv = Identifier::vecRGB2HSV(RGB);
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(Identifier::inThresh(COLOURS.at(i).h, hsv[0]) &&
           Identifier::inThresh(COLOURS.at(i).s, hsv[1]) &&
           Identifier::inThresh(COLOURS.at(i).v, hsv[2])) return COLOURS.at(i).colour;
    }
    return Colour::black;
}

Colour Identifier::identifyRGB(cv::Vec3b RGB)
{
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(RGB[0] == COLOURS.at(i).r &&
           RGB[1] == COLOURS.at(i).b &&
           RGB[2] == COLOURS.at(i).g) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

Colour Identifier::identifyRGB(uint8_t r, uint8_t g, uint8_t b)
{
    cv::Vec3b RGB(r, g, b);
    for(auto i = 0; i < COLOURS.size(); i++)
    {
        if(RGB[0] == COLOURS.at(i).r &&
           RGB[1] == COLOURS.at(i).b &&
           RGB[2] == COLOURS.at(i).g) return COLOURS.at(i).colour;
    }
    return Colour::white;
}

cv::Vec3b Identifier::vecRGB2HSV(cv::Vec3b rgb_vec)
{
    cv::Mat hsv;
    cv::Mat rgb(1,1, CV_8UC3, rgb_vec);
    cv::cvtColor(rgb, hsv, CV_RGB2HSV);
    return cv::Vec3b(hsv.data[0], hsv.data[1], hsv.data[2]);
}

cv::Vec3b Identifier::vecHSV2RGB(cv::Vec3b hsv_vec)
{
    cv::Mat rgb;
    cv::Mat hsv(1,1, CV_8UC3, hsv_vec);
    cv::cvtColor(hsv, rgb, CV_HSV2RGB);
    return cv::Vec3b(rgb.data[0], rgb.data[1], rgb.data[2]);
}

bool Identifier::inThresh(std::vector<uint8_t> thresh, uint8_t input)
{
    bool in_range = false;
    for(int i = 0; i < thresh.size(); i = i + 2)
    {
        if(input >= thresh.at(i) && input <= thresh.at(i+1)) in_range = true;
    }
    return in_range;
}

cv::Vec3b getRGB(Colour colour)
{
    for(auto i = 0; i < ALL_COLOURS.size(); i++)
    {
        if(colour == ALL_COLOURS.at(i).colour){
            return cv::Vec3b(ALL_COLOURS.at(i).r, ALL_COLOURS.at(i).g, ALL_COLOURS.at(i).b);
        }
    }
    return cv::Vec3b(0,0,0);
}
uint8_t getR(Colour colour)
{
    for(auto i = 0; i < ALL_COLOURS.size(); i++)
    {
        if(colour == ALL_COLOURS.at(i).colour){
            return ALL_COLOURS.at(i).r;
        }
    }
    return 0;
}
uint8_t getG(Colour colour)
{
    for(auto i = 0; i < ALL_COLOURS.size(); i++)
    {
        if(colour == ALL_COLOURS.at(i).colour){
            return ALL_COLOURS.at(i).g;
        }
    }
    return 0;
}
uint8_t getB(Colour colour)
{
    for(auto i = 0; i < ALL_COLOURS.size(); i++)
    {
        if(colour == ALL_COLOURS.at(i).colour){
            return ALL_COLOURS.at(i).b;
        }
    }
    return 0;
}

std::string getName(Colour colour)
{
    for(auto i = 0; i < ALL_COLOURS.size(); i++)
    {
        if(colour == ALL_COLOURS.at(i).colour){
            return ALL_COLOURS.at(i).name;
        }
    }
    return " ";
}
}
