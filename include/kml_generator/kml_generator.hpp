#ifndef KML_GENERATOR
#define KML_GENERATOR

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <sensor_msgs/NavSatFix.h>

namespace kml_utils
{

struct OtherInfo
{
  std::string name; // ex) name = "Velocity Scale Factor"
  std::string value_str; // ex) value_str = "0.987684"
};

struct Point
{
public:
  int seq = 0; // [sequence number]
  double time;      // [sec]
  double latitude;  // [deg]
  double longitude; // [deg]
  double altitude;  // [m]

  std::vector<OtherInfo> other_info_vector;
};

void addOtherInformation(Point & point, std::string other_info_name, std::string other_info_value_str);

std::string makeDouble2String(double d);
std::string makeBool2String(bool b);

}

class KmlGenerator
{
public:
  KmlGenerator();
  KmlGenerator(std::string file_name);
  KmlGenerator(std::string file_name, std::string logo_name, std::string log_link_url);

  enum class IntervalType
  {
    TIME_INTERBAL = 0,
    DISTANCE_INTERBAL = 1,
  };

  enum class KMLType
  {
    POINT = 0,
    LINE = 1,
  };

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

  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name, int visibility, ColorType ct);
  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, int visibility, ColorType ct);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name, int visibility, ColorType ct);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, int visibility, ColorType ct);

  bool addPointVector2LineKML(const std::vector<kml_utils::Point>&, std::string data_name, int visibility, ColorType ct);
  bool addPointVector2LineKML(const std::vector<kml_utils::Point>&, int visibility, ColorType ct);
  bool addPointVector2PointKML(const std::vector<kml_utils::Point>&, std::string data_name, int visibility, ColorType ct);
  bool addPointVector2PointKML(const std::vector<kml_utils::Point>&, int visibility, ColorType ct);

  bool outputKml();

  void setIntervalType(const IntervalType ip); // IntervalType::TIME_INTERBAL or IntervalType::DISTANCE_INTERBAL
  void setTimeInterval(const double time_interval); // [sec]
  void setPointInterval(const double point_interval); // [m]
  void setLineInterval(const double line_interval); // [m]
  IntervalType getIntervalType();
  double getTimeInterval();
  double getPointInterval();
  double getLineInterval();

private:
  // Variables
  std::string file_name_ = "output.kml";
  std::string log_link_url_;
  std::ofstream kml_file_ofs_;
  std::string header_;
  std::string body_;
  std::string footer_;
  int data_count_ = 0;

  IntervalType interval_type_ = IntervalType::TIME_INTERBAL; // time : 0, distance: 1
  KMLType kml_type_;
  std::string data_name_;
  ColorType color_type_;
  double time_interval_ = 0.2; // [sec]
  double point_interval_ = 5.0; // [m]
  double line_interval_ = 1.0; // [m]

  void initKml(std::string name);

  bool addKmlLineHeader(std::string data_name);
  bool addKmlLineBody(std::string data_name, std::string data_str, int visibility);
  bool addKmlPointBody(std::string data_name, std::string data_str, int visibility);

  void LLH2StringInCondition(std::string & str,double & time_last, double ecef_pose_last[3],
  const double time, double llh[3], int seq, double ecef_base_pose[3], std::vector<kml_utils::OtherInfo> other_info_vector);

  std::string NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>&);
  std::string LLHTimeSeq2PointStr(const int seq,const double time, double llh[3], const int sequence,
    std::vector<kml_utils::OtherInfo> other_info_vector);
  std::string NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);

  std::string PointVector2LineStr(const std::vector<kml_utils::Point>&);
  std::string PointVector2PointStr(const std::vector<kml_utils::Point>&, std::string data_name);

  std::string getColorCode();

  void llh2xyz(double*, double*);
  void xyz2enu(double ecef_pos[3], double ecef_base_pos[3], double enu_pos[3]);
  void ecef2llh(double ecef_pos[3], double llh_pos[3]);

};

#endif
