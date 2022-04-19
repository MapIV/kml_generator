#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>

#include <kml_generator/kml_generator.hpp>

std::vector<std::string> split(std::string &input, char delimiter) {
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

ros::Time str2ROSTime(std::string input)
{
  ros::Time time;
  time.sec = std::stoi(input.substr(0, 10));
  time.nsec = std::stoi(input.substr(10, 9));
  return time;
}

std::vector<PointInfomation> EagleyeLogCsvtoVector(std::string csv_name)
{
  std::ifstream ifs(csv_name);
  if (!ifs)
  {
    std::cerr << "Input CSV File Not Found!" << '\n';
    exit(1);
  }

  std::vector<PointInfomation> vector_eagleye;
  int cnt = 0;
  std::string line;
  while (getline(ifs, line))
  {

    if (cnt == 0)
    {
      cnt++;
      continue;
    }
    PointInfomation tmp_eagleye;

    std::vector<std::string> str_vec = split(line, ',');

    tmp_eagleye.seq = cnt;
    tmp_eagleye.time = str2ROSTime(str_vec.at(0)).toSec();
    tmp_eagleye.latitude = std::stod(str_vec.at(107));
    tmp_eagleye.longitude = std::stod(str_vec.at(108));
    tmp_eagleye.altitude = std::stod(str_vec.at(109));

    vector_eagleye.push_back(tmp_eagleye);

    cnt++;

  }
  
  return vector_eagleye;
}

int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: eagleyelogcsv2kml input.csv output.kml" << std::endl;
    return 0;
  }

  std::string csv_name = argv[1];
  std::string kml_name = argv[2];

  std::vector<PointInfomation> vector_point_information = EagleyeLogCsvtoVector(csv_name);

  std::string logo_name = "eagleye";
  std::string logo_link_url = "https://github.com/MapIV/eagleye/blob/main-ros1/docs/logo.png";
  KmlGenerator kml_generator(kml_name, logo_name, logo_link_url);

  // kml_generator.setTimeInterval(0.1);
  // kml_generator.addPointInfomationVectorLine(vector_point_information, "Eagleye Line");
  kml_generator.setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);
  kml_generator.setLineInterval(2.0);
  kml_generator.addPointInfomationVectorPoint(vector_point_information, "Eagleye Point1");
  // kml_generator.addPointInfomationVectorPoint(vector_point_information, "Eagleye Point2");

  kml_generator.outputKml();

  return 0;
}
