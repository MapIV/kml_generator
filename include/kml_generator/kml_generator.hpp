#ifndef KML_GENERATOR
#define KML_GENERATOR

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <sensor_msgs/NavSatFix.h>

class KmlGenerator
{
public:
  KmlGenerator();
  KmlGenerator(std::string file_name);
  KmlGenerator(std::string file_name, std::string logo_name, std::string log_link_url);

  void initKml(std::string name);

  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);
  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&);

  bool outputKml();

  enum class IntervalType
  {
    TIME_INTERBAL = 0,
    DISTANCE_INTERBAL = 1,
  };

  void setIntervalType(const IntervalType i);
  void setTimeInterval(const double time_interval);
  void setPointInterval(const double point_interval);
  void setLineInterval(const double line_interval);

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
  double time_interval_ = 0.2; // [sec]
  double point_interval_ = 1.0; // [m]
  double line_interval_ = 1.0; // [m]

  std::string color_list[8] 
    = {"ff0000ff","ff00ff00","ffff0000","ffff55aa","ffffff00","ff7700ff","ff00aaff","ffffffff"};
    //    Red        Green      Blue      Purple      Cyan      Magenta    Orange      White

  bool addKmlLineHeader(std::string data_name);
  bool addKmlLineBody(std::string data_name, std::string data_str);
  bool addKmlPointBody(std::string data_name, std::string data_str);
  std::string NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>&);
  std::string NavSatFixMsg2PointStr(const sensor_msgs::NavSatFix, std::string data_name);
  std::string NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);

  void llh2xyz(double*, double*);

};

#endif
