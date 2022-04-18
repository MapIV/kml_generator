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

  bool addNavSatFixMsg(std::vector<sensor_msgs::NavSatFix>, std::string data_name);
  bool addNavSatFixMsg(std::vector<sensor_msgs::NavSatFix>);
  bool addKmlHeader(std::string data_name);
  bool addKmlBody(std::string data_name, std::string data_str);
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
  
  std::string NavSatFixMsgtoStr(std::vector<sensor_msgs::NavSatFix>&);

};

#endif
