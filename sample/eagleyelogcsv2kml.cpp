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

std::vector<kml_utils::Point> EagleyeLogCsvtoVector(std::string csv_name)
{
  std::ifstream ifs(csv_name);
  if (!ifs)
  {
    std::cerr << "Input CSV File Not Found!" << '\n';
    exit(1);
  }

  std::vector<kml_utils::Point> vector_eagleye;
  int cnt = 0;
  std::string line;
  while (getline(ifs, line))
  {

    if (cnt == 0)
    {
      cnt++;
      continue;
    }
    kml_utils::Point tmp_eagleye;

    std::vector<std::string> str_vec = split(line, ',');

    tmp_eagleye.seq = cnt;
    tmp_eagleye.time = str2ROSTime(str_vec.at(0)).toSec();
    tmp_eagleye.latitude = std::stod(str_vec.at(107));
    tmp_eagleye.longitude = std::stod(str_vec.at(108));
    tmp_eagleye.altitude = std::stod(str_vec.at(109));

    kml_utils::OtherInfo other_info0;
    other_info0.name = "Velocity Scale Factor";
    other_info0.value_str = str_vec.at(25);

    kml_utils::OtherInfo other_info1;
    other_info1.name = "Velocity Scale Factor Flag";
    other_info1.value_str = str_vec.at(32);

    tmp_eagleye.other_info_vector.push_back(other_info0);
    tmp_eagleye.other_info_vector.push_back(other_info1);
  
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

  std::vector<kml_utils::Point> vector_point = EagleyeLogCsvtoVector(csv_name);

  std::string legend_name = "eagleye";
  std::string logo_link_url = "https://github.com/MapIV/eagleye/blob/main-ros1/docs/logo.png";
  KmlGenerator kml_generator(kml_name, legend_name, logo_link_url); // or  KmlGenerator(kml_name);

  int visibility = 1;

  kml_generator.setTimeInterval(1.0);
  kml_generator.addPointVector2LineKML(vector_point, "Eagleye Line", visibility, KmlGenerator::ColorType::RED);

  kml_generator.setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);

  kml_generator.addPointVector2PointKML(vector_point, "Eagleye Point1", visibility, KmlGenerator::ColorType::BLUE);

  kml_generator.setPointInterval(10.0);
  kml_generator.addPointVector2PointKML(vector_point, "Eagleye Point2", visibility, KmlGenerator::ColorType::GREEN);

  kml_generator.outputKml();

  return 0;
}
