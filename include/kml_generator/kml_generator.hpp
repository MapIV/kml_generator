#ifndef KML_GENERATOR
#define KML_GENERATOR

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <sensor_msgs/NavSatFix.h>

struct Point
{
public:
  int seq = 0; // [sequence number]
  double time;      // [sec]
  double latitude;  // [deg]
  double longitude; // [deg]
  double altitude;  // [m]
  struct OtherInfo
  {
    std::string name; // ex) name = "Velocity Scale Factor"
    std::string value_str; // ex) value_str = "0.987684"
  };
  std::vector<OtherInfo> other_info_vector;
};

class KmlGenerator
{
public:
  KmlGenerator();
  KmlGenerator(std::string file_name);
  KmlGenerator(std::string file_name, std::string logo_name, std::string log_link_url);

  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name, int visibility);
  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, int visibility);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name, int visibility);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, int visibility);

  bool addPointVector2LineKML(const std::vector<Point>&, std::string data_name, int visibility);
  bool addPointVector2LineKML(const std::vector<Point>&, int visibility);
  bool addPointVector2PointKML(const std::vector<Point>&, std::string data_name, int visibility);
  bool addPointVector2PointKML(const std::vector<Point>&, int visibility);

  bool outputKml();

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

  void setIntervalType(const IntervalType ip); // IntervalType::TIME_INTERBAL or IntervalType::DISTANCE_INTERBAL
  void setTimeInterval(const double time_interval); // [sec]
  void setPointInterval(const double point_interval); // [m]
  void setLineInterval(const double line_interval); // [m]

  std::string make_double_string(double d);

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
  double time_interval_ = 0.2; // [sec]
  double point_interval_ = 5.0; // [m]
  double line_interval_ = 1.0; // [m]

  std::string color_list[8] 
    = {"ff0000ff","ff00ff00","ffff0000","ffff55aa","ffffff00","ff7700ff","ff00aaff","ffffffff"};
    //    Red        Green      Blue      Purple      Cyan      Magenta    Orange      White

  void initKml(std::string name);

  bool addKmlLineHeader(std::string data_name);
  bool addKmlLineBody(std::string data_name, std::string data_str, int visibility);
  bool addKmlPointBody(std::string data_name, std::string data_str, int visibility);

  void LLH2StringInCondition(std::string & str,double & time_last, double ecef_pose_last[3],
  const double time, double llh[3], int seq, double ecef_base_pose[3], std::vector<Point::OtherInfo> other_info_vector);

  std::string NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>&);
  std::string LLHTimeSeq2PointStr(const int seq,const double time, double llh[3], const int sequence,
    std::vector<Point::OtherInfo> other_info_vector);
  std::string NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);

  std::string PointVector2LineStr(const std::vector<Point>&);
  std::string PointVector2PointStr(const std::vector<Point>&, std::string data_name);

  void llh2xyz(double*, double*);
  void xyz2enu(double ecef_pos[3], double ecef_base_pos[3], double enu_pos[3]);
  void ecef2llh(double ecef_pos[3], double llh_pos[3]);

};

#endif
