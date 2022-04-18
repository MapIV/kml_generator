#include <kml_generator/kml_generator.hpp>

KmlGenerator::KmlGenerator()
{
  kml_file_ofs_.open(file_name_, std::ios::out);
  std::cout << "Output file = " << file_name_ << std::endl;
  initKml(file_name_);
}

KmlGenerator::KmlGenerator(std::string file_name)
{
  file_name_ = file_name;
  kml_file_ofs_.open(file_name_, std::ios::out);
  std::cout << "Output file = " << file_name_ << std::endl;
  initKml(file_name_);
}

KmlGenerator::KmlGenerator(std::string file_name, std::string log_link_url)
{
  file_name_ = file_name;
  log_link_url_ = log_link_url;
  kml_file_ofs_.open(file_name_, std::ios::out);
  std::cout << "Output file = " << file_name_ << std::endl;
  initKml(file_name_);

}

void KmlGenerator::initKml(std::string name)
{
   header_ =
      "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      "<kml xmlns=\"http://earth.google.com/kml/2.2\">\n"
      "<Document>\n"
      "<name>"+ name +"</name>\n\n"
      +(
      log_link_url_.empty() ? "":
      "<ScreenOverlay>\n"
      "\t<name>"+ name +" logo</name>\n"
      "\t<visibility>1</visibility>\n"
      "\t<Icon>\n"
      + "\t\t<href>" + log_link_url_ + "?raw=true</href>\n" +
      "\t</Icon>\n"
      "\t<overlayXY x=\"-0.3\" y=\"-1\" xunits=\"fraction\" yunits=\"fraction\"/>\n"
      "\t<screenXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>\n"
      "\t<size x=\"260.6\" y=\"56.0\" xunits=\"pixels\" yunits=\"pixels\"/>\n"
      "</ScreenOverlay>\n\n"
      )
      ;
    
   footer_ =
      "</Document>\n"
      "</kml>\n";
}

bool KmlGenerator::addKmlHeader(std::string data_name)
{
  std::string tmp_header;
  std::string color_str = color_list[(data_count_-1)%8];
  tmp_header =
      "<Style id=\" " + data_name + "\">\n"
      "\t<LineStyle>\n"
      "\t\t<color> " + color_str + " </color>\n"
      "\t\t<width>5.00</width>\n"
      "\t</LineStyle>\n"
      "</Style>\n\n";
  header_ +=tmp_header;

  return true;
}

bool KmlGenerator::addKmlBody(std::string data_name, std::string data_str)
{
  std::string tmp_body;
  tmp_body =
      "\t<Placemark>\n"
      "\t<name> " + data_name + " </name>\n"
      "\t<visibility>1</visibility>\n"
      "\t<description><![CDATA[]]></description>\n"
      "\t<styleUrl># " + data_name + "</styleUrl>\n"
      "\t<LineString>\n"
      "\t\t<extrude>0</extrude>\n"
      "\t\t<tessellate>1</tessellate>\n"
      "\t\t<altitudeMode>clampToGround</altitudeMode>\n"
      "\t\t<coordinates>\n"
      + data_str +
      "\t\t</coordinates>\n"
      "\t</LineString>\n"
      "\t</Placemark>\n\n";
  body_ +=tmp_body;

  return true;
}

std::string KmlGenerator::NavSatFixMsgtoStr(std::vector<sensor_msgs::NavSatFix>& fix_msg)
{
  std::stringstream data_ss;
  std::size_t data_length = std::distance(fix_msg.begin(), fix_msg.end());

  double time_last = 0;
  for (int i=0; i<data_length; i++)
  {
    if(fix_msg[i].header.stamp.toSec() != time_last
      && fix_msg[i].header.stamp.toSec() - time_last > 0.2
      && fix_msg[i].longitude != 0 && fix_msg[i].latitude != 0)
    {
      data_ss << std::setprecision(std::numeric_limits<double>::max_digits10) 
        << fix_msg[i].longitude << "," 
        << fix_msg[i].latitude << "," 
        << fix_msg[i].altitude 
        << std::endl;
      time_last = fix_msg[i].header.stamp.toSec();
    }
  }

  return data_ss.str();
}

bool KmlGenerator::addNavSatFixMsg(std::vector<sensor_msgs::NavSatFix> fix_msg)
{
  std::string data_str = NavSatFixMsgtoStr(fix_msg);
  std::string data_name;

  data_count_++;
  data_name = "DataNum " + std::to_string(data_count_);

  if (!addKmlHeader(data_name)) return false;
  if (!addKmlBody(data_name,data_str)) return false;
  return true;
}

bool KmlGenerator::addNavSatFixMsg(std::vector<sensor_msgs::NavSatFix> fix_msg, std::string data_name)
{
  std::string data_str = NavSatFixMsgtoStr(fix_msg);

  data_count_++;
  if (!addKmlHeader(data_name)) return false;
  if (!addKmlBody(data_name, data_str)) return false;
  return true;
}

bool KmlGenerator::outputKml()
{
  kml_file_ofs_ << header_ << body_ <<  footer_ << std::endl;

  return true;
}