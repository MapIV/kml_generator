#ifndef KML_GENERATOR
#define KML_GENERATOR

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <sensor_msgs/NavSatFix.h>

struct PointInfomation
{
public:
  int seq = 0;
  double time;
  double latitude;
  double longitude;
  double altitude;
  struct OtherInfo
  {
    std::string name;
    std::string value_str;
  };
  std::vector<OtherInfo> other_info;
};

class KmlGenerator
{
public:
  KmlGenerator();
  KmlGenerator(std::string file_name);
  KmlGenerator(std::string file_name, std::string logo_name, std::string log_link_url);

  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);
  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&);

  bool addPointInfomationVectorLine(const std::vector<PointInfomation>&, std::string data_name);
  bool addPointInfomationVectorLine(const std::vector<PointInfomation>&);
  bool addPointInfomationVectorPoint(const std::vector<PointInfomation>&, std::string data_name);
  bool addPointInfomationVectorPoint(const std::vector<PointInfomation>&);

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

  void setIntervalType(const IntervalType ip);
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
  KMLType kml_type_;
  std::string data_name_;
  double time_interval_ = 0.2; // [sec]
  double point_interval_ = 1.0; // [m]
  double line_interval_ = 1.0; // [m]

  std::string color_list[8] 
    = {"ff0000ff","ff00ff00","ffff0000","ffff55aa","ffffff00","ff7700ff","ff00aaff","ffffffff"};
    //    Red        Green      Blue      Purple      Cyan      Magenta    Orange      White

  void initKml(std::string name);

  bool addKmlLineHeader(std::string data_name);
  bool addKmlLineBody(std::string data_name, std::string data_str);
  bool addKmlPointBody(std::string data_name, std::string data_str);

  void LLH2StringInCondition(std::string & str,double & time_last, double ecef_pose_last[3],
  const double time, double llh[3], int seq);

  std::string NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>&);
  std::string NavSatFixMsg2PointStr(const int seq,const double time, double llh[3], const int sequence);
  std::string NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);

  std::string PointInfomationVector2LineStr(const std::vector<PointInfomation>&);
  std::string PointInfomationVector2PointStr(const std::vector<PointInfomation>&, std::string data_name);

  void llh2xyz(double*, double*);

};

#endif
