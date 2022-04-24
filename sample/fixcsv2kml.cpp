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

std::vector<sensor_msgs::NavSatFix> NavSatFixMsgCsvtoVector(std::string csv_name)
{
  std::ifstream ifs(csv_name);
  if (!ifs)
  {
    std::cerr << "Input CSV File Not Found!" << '\n';
    exit(1);
  }

  std::vector<sensor_msgs::NavSatFix> vector_fix;
  int cnt = 0;
  std::string line;
  while (getline(ifs, line))
  {

    if (cnt == 0)
    {
      cnt++;
      continue;
    }
    sensor_msgs::NavSatFix tmp_fix;

    std::vector<std::string> str_vec = split(line, ',');

    tmp_fix.header.seq = std::stoi(str_vec.at(1));
    tmp_fix.header.stamp = str2ROSTime(str_vec.at(2));
    tmp_fix.header.frame_id = str_vec.at(3);

    tmp_fix.status.status = std::stoi(str_vec.at(4));
    tmp_fix.status.service = std::stoi(str_vec.at(5));
 
    tmp_fix.latitude = std::stod(str_vec.at(6));
    tmp_fix.longitude = std::stod(str_vec.at(7));
    tmp_fix.altitude= std::stod(str_vec.at(8));

    tmp_fix.position_covariance[0] = std::stof(str_vec.at(9));
    tmp_fix.position_covariance[1] = std::stof(str_vec.at(10));
    tmp_fix.position_covariance[2] = std::stof(str_vec.at(11));
    tmp_fix.position_covariance[3] = std::stof(str_vec.at(12));
    tmp_fix.position_covariance[4] = std::stof(str_vec.at(13));
    tmp_fix.position_covariance[5] = std::stof(str_vec.at(14));
    tmp_fix.position_covariance[6] = std::stof(str_vec.at(15));
    tmp_fix.position_covariance[7] = std::stof(str_vec.at(16));
    tmp_fix.position_covariance[8] = std::stof(str_vec.at(17));
    tmp_fix.position_covariance_type = std::stoi(str_vec.at(18));

    vector_fix.push_back(tmp_fix);

    cnt++;

  }
  
  return vector_fix;
}

int main(int argc, char** argv) {
  if(argc != 3) {
    std::cout << "usage: fixcsv2kml input.csv output.kml" << std::endl;
    return 0;
  }

  std::string csv_name = argv[1];
  std::string kml_name = argv[2];

  std::vector<sensor_msgs::NavSatFix> vector_fix = NavSatFixMsgCsvtoVector(csv_name);

  std::string legend_name = "eagleye";
  std::string logo_link_url = "https://github.com/MapIV/eagleye/blob/main-ros1/docs/logo.png";
  KmlGenerator kml_generator(kml_name, legend_name, logo_link_url); // or  KmlGenerator(kml_name);

  int visibility = 1;

  kml_generator.setTimeInterval(0.1);
  kml_generator.addNavSatFixMsgVectorLine(vector_fix, "GNSS Line", visibility, KmlGenerator::ColorType::RED);

  kml_generator.addNavSatFixMsgVectorPoint(vector_fix, "GNSS Point1", visibility, KmlGenerator::ColorType::BLUE);

  kml_generator.setIntervalType(KmlGenerator::IntervalType::DISTANCE_INTERBAL);
  kml_generator.setPointInterval(10.0);
  kml_generator.addNavSatFixMsgVectorPoint(vector_fix, "GNSS Point2", visibility, KmlGenerator::ColorType::GREEN);

  kml_generator.outputKml();

  return 0;
}
