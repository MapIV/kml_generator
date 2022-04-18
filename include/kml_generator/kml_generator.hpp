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
  KmlGenerator(std::string file_name, std::string log_link_url);

  void initKml(std::string name);

  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);
  bool addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>&);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);
  bool addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>&);
  bool addKmlLineHeader(std::string data_name);
  bool addKmlPointHeader(std::string data_name);
  bool addKmlLineBody(std::string data_name, std::string data_str);
  bool addKmlPointBody(std::string data_str);
  bool outputKml();

private:
  // Variables
  std::string file_name_ = "output.kml";
  std::string log_link_url_;
  std::ofstream kml_file_ofs_;
  std::string header_;
  std::string body_;
  std::string footer_;
  int data_count_ = 0;

  std::string color_list[8] 
    = {"ff0000ff","ff00ff00","ffff0000","ffff55aa","ffffff00","ff7700ff","ff00aaff","ffffffff"};
    //    Red        Green      Blue      Purple      Cyan      Magenta    Orange      White
  
  std::string NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>&);
  std::string NavSatFixMsg2PointStr(const sensor_msgs::NavSatFix, std::string data_name);
  std::string NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>&, std::string data_name);

};

#endif
