#ifndef KML_GENERATOR_COMMON_HPP
#define KML_GENERATOR_COMMON_HPP

#include <string>
#include <vector>

namespace kml_utils
{
enum class ColorType
{
  RED = 0,
  GREEN = 1,
  BLUE = 2,
  PURPLE = 3,
  CYAN = 4,
  MAGENTA = 5,
  ORANGE = 6,
  WHITE = 7,
};

constexpr const char* line_style_red = "LineStyleRED";
constexpr const char* line_style_green = "LineStyleGREEN";
constexpr const char* line_style_blue = "LineStyleBLUE";
constexpr const char* line_style_purple = "LineStylePURPLE";
constexpr const char* line_style_cyan = "LineStyleCYAN";
constexpr const char* line_style_magenta = "LineStyleMAGENTA";
constexpr const char* line_style_orange = "LineStyleORANGE";
constexpr const char* line_style_white = "LineStyleWHITE";

struct OtherInfo
{
  std::string name;       // ex) name = "Velocity Scale Factor"
  std::string value_str;  // ex) value_str = "0.987684"
};

struct Header
{
  std::string name;
  std::string description;
};

struct Point
{
  int seq = 0;       // [sequence number]
  double time;       // [sec]
  double latitude;   // [deg]
  double longitude;  // [deg]
  double altitude;   // [m]

  std::vector<OtherInfo> other_info_vector;
};

struct Line
{
  Header header;
  std::vector<Point> points;
  ColorType color;

  Line()
  {
  }
  Line(const std::string& name, const std::string& description, const ColorType color)
  {
    header.name = name;
    header.description = description;
    this->color = color;
  }
};

struct Lines
{
  Header header;
  std::vector<Line> lines;
};
}  // namespace kml_utils

#endif
