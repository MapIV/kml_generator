#include <kml_generator/kml_generator.hpp>

namespace kml_utils
{

void addOtherInformation(Point & point, std::string other_info_name, std::string other_info_value_str)
{
  OtherInfo other_info;
  other_info.name = other_info_name;
  other_info.value_str = other_info_value_str;
  point.other_info_vector.push_back(other_info);
};

std::string makeDouble2String(double d)
{
  std::stringstream s;
  s.precision(std::numeric_limits<double>::max_digits10);
  s << std::scientific << d;
  return s.str();
}

std::string makeBool2String(bool b)
{
  std::string str = b ?  "True" : "False";
  return str;
}

}

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

KmlGenerator::KmlGenerator(std::string file_name, std::string logo_name, std::string log_link_url)
{
  file_name_ = file_name;
  log_link_url_ = log_link_url;
  kml_file_ofs_.open(file_name_, std::ios::out);
  std::cout << "Output file = " << file_name_ << std::endl;
  initKml(logo_name);

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

bool KmlGenerator::addKmlLineHeader(std::string data_name)
{
  std::string tmp_header;
  std::string color_str = getColorCode();
  tmp_header =
      "<Style id=\"" + data_name + "\">\n"
      "\t<LineStyle>\n"
      "\t\t<color> " + color_str + " </color>\n"
      "\t\t<width>5.00</width>\n"
      "\t</LineStyle>\n"
      "</Style>\n\n";
  header_ +=tmp_header;

  return true;
}

bool KmlGenerator::addKmlLineBody(std::string data_name, std::string data_str, int visibility)
{
  std::string tmp_body;
  tmp_body =
      "\t<Placemark>\n"
      "\t<name> " + data_name + " </name>\n"
      "\t<visibility>"+std::to_string(visibility)+"</visibility>\n"
      "\t<description><![CDATA[]]></description>\n"
      "\t<styleUrl>#" + data_name + "</styleUrl>\n"
      "\t<LineString>\n"
      "\t\t<extrude>0</extrude>\n"
      "\t\t<tessellate>1</tessellate>\n"
      "\t\t<altitudeMode>clampToGround</altitudeMode>\n"
      "\t\t<coordinates>\n"
      + data_str +
      "\t\t</coordinates>\n"
      "\t</LineString>\n"
      "\t</Placemark>\n\n";
  body_ += tmp_body;

  return true;
}

bool KmlGenerator::addKmlPointBody(std::string data_name, std::string data_str, int visibility)
{
  std::string color_str = getColorCode();
  body_ +=  
          "<open>1</open>\n"
         "\t<Style id=\"" + data_name + "\">\n"
          "\t\t<IconStyle>\n"
          "\t\t\t<color>"+ color_str +"</color>\n"
          "\t\t\t<scale>0.5</scale>\n"
          "\t\t\t<Icon>\n"
          "\t\t\t<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n"
          "\t\t\t</Icon>\n"
          "\t\t</IconStyle>\n"
          "\t\t<LabelStyle>\n"
          "\t\t\t<scale>0</scale>\n"
          "\t\t</LabelStyle>\n"
          "\t\t<BalloonStyle>\n"
          "\t\t\t<bgColor>fffff8f0</bgColor>\n"
          "\t\t\t<text><![CDATA[<b><font color=\"#CC0000\" size=\"+3\">$[name]</font></b><br>$[description]</font><br/>]]></text>\n"
          "\t\t</BalloonStyle>\n"
          "\t</Style>\n"
          "<Folder id=\"ID2\">\n"
          "\t<name>" + data_name + "</name>\n"
          "\t<visibility>"+ std::to_string(visibility) +"</visibility>\n"
          "\t<open>0</open>"
          ;
  body_ += data_str;
  body_ += "\t</Folder>\n";

  return true;
}

void KmlGenerator::LLH2StringInCondition(std::string & str,double & time_last, double ecef_pose_last[3],
  const double time, double llh[3], int seq, double ecef_base_pose[3], std::vector<kml_utils::OtherInfo> other_info_vector)
{
    bool update_flag = false;
    double ecef_pose[3];
    double enu_pose[3];
    double enu_pose_last[3];
    if(interval_type_ == IntervalType::TIME_INTERBAL)
    {
      update_flag = (time != time_last && time - time_last > time_interval_);
    }
    else if (interval_type_ == IntervalType::DISTANCE_INTERBAL)
    {
      double llh_pos[3] = {llh[0] * M_PI/180, llh[1] * M_PI/180, llh[2] * M_PI/180};
      llh2xyz(llh_pos, ecef_pose);

      if(ecef_pose_last[0]==0 && ecef_pose_last[1] == 0 && ecef_pose_last[2] == 0)
      {
        ecef_base_pose[0] = ecef_pose[0];
        ecef_base_pose[1] = ecef_pose[1];
        ecef_base_pose[2] = ecef_pose[2];
      }

      xyz2enu(ecef_pose, ecef_base_pose, enu_pose);
      xyz2enu(ecef_pose_last, ecef_base_pose, enu_pose_last);

      double diff_pose[2] ={enu_pose[0] - enu_pose_last[0], enu_pose[1] - enu_pose_last[1]};
      double distance = sqrt(pow(diff_pose[0], 2.0) + pow(diff_pose[1], 2.0));
      
      if (kml_type_ ==  KMLType::LINE)
      {
        update_flag = distance > line_interval_;
      }
      else if (kml_type_ ==  KMLType::POINT)
      {
        update_flag = distance > point_interval_;
      }
    }

    if(update_flag && llh[0] != 0 && llh[1] != 0)
    {
      if(kml_type_ ==  KMLType::LINE)
      {
        std::stringstream data_ss;
        data_ss <<
          std::setprecision(std::numeric_limits<double>::max_digits10) << llh[1] << "," <<
          std::setprecision(std::numeric_limits<double>::max_digits10) << llh[0] << "," <<
          std::setprecision(std::numeric_limits<double>::max_digits10) << llh[2]
          << std::endl;
        str = data_ss.str();
      }
      else if (kml_type_ ==  KMLType::POINT)
      {
        str += LLHTimeSeq2PointStr(seq, time, llh, seq, other_info_vector);
      }
      
      time_last = time;
      ecef_pose_last[0] = ecef_pose[0];
      ecef_pose_last[1] = ecef_pose[1];
      ecef_pose_last[2] = ecef_pose[2];
    }
  return;
}

std::string KmlGenerator::NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector)
{
  std::stringstream data_ss;
  std::size_t data_length = fix_msg_vector.size();

  double time_last = 0;
  double ecef_pose_last[3] = {0, 0, 0};
  double ecef_base_pose[3] = {0, 0, 0};
  for (int i=0; i<data_length; i++)
  {
    std::vector<kml_utils::OtherInfo> other_info_vector;
    double time = fix_msg_vector[i].header.stamp.toSec();
    double llh[3] = {fix_msg_vector[i].latitude, fix_msg_vector[i].longitude, fix_msg_vector[i].altitude};
    int seq = fix_msg_vector[i].header.seq;
    std::string str;
    LLH2StringInCondition(str,time_last,ecef_pose_last, time, llh, seq, ecef_base_pose, other_info_vector);
    data_ss << str;
  }

  return data_ss.str();
}

std::string KmlGenerator::PointVector2LineStr(const std::vector<kml_utils::Point> & point_vector)
{
  std::stringstream data_ss;
  std::size_t data_length = point_vector.size();

  double time_last = 0;
  double ecef_pose_last[3] = {0, 0, 0};
  double ecef_base_pose[3] = {0, 0, 0};
  for (int i=0; i<data_length; i++)
  {
    std::vector<kml_utils::OtherInfo> other_info_vector = point_vector[i].other_info_vector;
    double time = point_vector[i].time;
    double llh[3] = {point_vector[i].latitude, point_vector[i].longitude, point_vector[i].altitude};
    int seq = point_vector[i].seq;
    std::string str;
    LLH2StringInCondition(str,time_last,ecef_pose_last, time, llh, seq, ecef_base_pose, other_info_vector);
    data_ss << str;
  }

  return data_ss.str();
}

std::string KmlGenerator::LLHTimeSeq2PointStr(const int seq,const double time, double llh[3], const int sequence,
  std::vector<kml_utils::OtherInfo> other_info_vector)
{
  std::string data;
  data =
  "\t<Placemark>\n\
                \t\t<name>Point</name>\n\
                \t\t<Snippest></Snippest>\n\
                <description><![CDATA[<B>Point Status</B><BR><BR>\n\
                \t\t\t<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n\
                \t\t\t\t<TR ALIGN=RIGHT>\n\
                \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Sequence Number: </TD><TD>" + std::to_string(sequence) + "</TD></TR>\n"
                +
                "\t\t\t\t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Timestamp: </TD><TD>"
            + kml_utils::makeDouble2String(time) + "</TD></TR>\n\
                 \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>lat: </TD><TD>"
            + kml_utils::makeDouble2String(llh[0]) + "</TD></TR>\n\
                \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>lon: </TD><TD>"
            + kml_utils::makeDouble2String(llh[1]) + "</TD></TR>\n\
                \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>height: </TD><TD>"
            + kml_utils::makeDouble2String(llh[2]) + "</TD></TR>\n";
  if(!other_info_vector.empty())
  {
    for(kml_utils::OtherInfo oi : other_info_vector){
      data += "\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>"+ oi.name+": </TD><TD>"
            + oi.value_str + "</TD></TR>\n";
    }
  }
  data +=   "\t\t\t</TABLE>\n"
                "]]></description>\n"
                "\t\t<styleUrl>#" + data_name_ +
            + "</styleUrl>\n"
                "\t\t<Point>\n"
                "t\t\t<coordinates>"
            + kml_utils::makeDouble2String(llh[1]) + ","
            + kml_utils::makeDouble2String(llh[0]) + ","
            + kml_utils::makeDouble2String(llh[2]) + ","
            + "</coordinates>\n"
                "\t\t</Point>\n"
                "\t</Placemark>";
  return data;
}

std::string KmlGenerator::NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, std::string data_name)
{
  std::string data;
  std::size_t data_length = std::distance(fix_msg_vector.begin(), fix_msg_vector.end());

  double time_last = 0;
  double ecef_pose_last[3] = {0, 0, 0};
  double ecef_base_pose[3] = {0, 0, 0};
  for (int i=0; i<data_length; i++)
  {
    std::vector<kml_utils::OtherInfo> other_info_vector;
    double time = fix_msg_vector[i].header.stamp.toSec();
    double llh[3] = {fix_msg_vector[i].latitude, fix_msg_vector[i].longitude, fix_msg_vector[i].altitude};
    int seq = fix_msg_vector[i].header.seq;
    std::string str;
    LLH2StringInCondition(str,time_last,ecef_pose_last, time, llh, seq, ecef_base_pose, other_info_vector);
    data += str;
  }

  return data;
}

std::string KmlGenerator::PointVector2PointStr(const std::vector<kml_utils::Point> & point_vector, std::string data_name)
{
  std::string data;
  std::size_t data_length = point_vector.size();

  double time_last = 0;
  double ecef_pose_last[3] = {0, 0, 0};
  double ecef_base_pose[3] = {0, 0, 0};
  for (int i=0; i<data_length; i++)
  {
    std::vector<kml_utils::OtherInfo> other_info_vector = point_vector[i].other_info_vector;
    double time = point_vector[i].time;
    double llh[3] = {point_vector[i].latitude, point_vector[i].longitude, point_vector[i].altitude};
    int seq = point_vector[i].seq;
    std::string str;
    LLH2StringInCondition(str,time_last,ecef_pose_last, time, llh, seq, ecef_base_pose, other_info_vector);
    data += str;
  }

  return data;
}


bool KmlGenerator::addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, int visibility, ColorType ct)
{
  std::string data_str = NavSatFixMsgVector2LineStr(fix_msg_vector);
  std::string data_name;

  data_name = "DATANUM_" + std::to_string(data_count_);

  return addNavSatFixMsgVectorLine(fix_msg_vector, data_name, visibility, ct);
}

bool KmlGenerator::addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, std::string data_name, int visibility, ColorType ct)
{
  kml_type_ =  KMLType::LINE;
  color_type_ = ct;

  data_name.erase(std::remove(data_name.begin(), data_name.end(), ' '), data_name.end());
  std::string data_str = NavSatFixMsgVector2LineStr(fix_msg_vector);

  data_count_++;
  if (!addKmlLineHeader(data_name)) return false;
  if (!addKmlLineBody(data_name, data_str, visibility)) return false;
  return true;
}

bool KmlGenerator::addPointVector2LineKML(const std::vector<kml_utils::Point> & point_vector, int visibility, ColorType ct)
{
  std::string data_str = PointVector2LineStr(point_vector);
  std::string data_name;

  data_name = "DATANUM_" + std::to_string(data_count_);

  return addPointVector2LineKML(point_vector, data_name, visibility, ct);
}

bool KmlGenerator::addPointVector2LineKML(const std::vector<kml_utils::Point> & point_vector, std::string data_name, int visibility, ColorType ct)
{
  kml_type_ =  KMLType::LINE;
  color_type_ = ct;

  data_name.erase(std::remove(data_name.begin(), data_name.end(), ' '), data_name.end());
  std::string data_str = PointVector2LineStr(point_vector);

  data_count_++;
  if (!addKmlLineHeader(data_name)) return false;
  if (!addKmlLineBody(data_name, data_str, visibility)) return false;
  return true;
}

bool KmlGenerator::addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, int visibility, ColorType ct)
{

  std::string data_name;

  data_name = "DATANUM_" + std::to_string(data_count_);

  return addNavSatFixMsgVectorPoint(fix_msg_vector, data_name, visibility, ct);
}

bool KmlGenerator::addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, std::string data_name, int visibility, ColorType ct)
{
  kml_type_ =  KMLType::POINT;
  color_type_ = ct;

  data_name.erase(std::remove(data_name.begin(), data_name.end(), ' '), data_name.end());
  data_name_ = data_name;

  data_count_++;

  std::string data_str = NavSatFixMsgVector2PointStr(fix_msg_vector, data_name);

  if (!addKmlPointBody(data_name, data_str, visibility)) return false;
  return true;
}

bool KmlGenerator::addPointVector2PointKML(const std::vector<kml_utils::Point> & point_vector, int visibility, ColorType ct)
{
  std::string data_name;

  data_name = "DATANUM_" + std::to_string(data_count_);

  return addPointVector2PointKML(point_vector, data_name, visibility, ct);
}

bool KmlGenerator::addPointVector2PointKML(const std::vector<kml_utils::Point> & point_vector, std::string data_name, int visibility, ColorType ct)
{
  kml_type_ =  KMLType::POINT;
  color_type_ = ct;

  data_name.erase(std::remove(data_name.begin(), data_name.end(), ' '), data_name.end());
  data_name_ = data_name;

  data_count_++;

  std::string data_str = PointVector2PointStr(point_vector, data_name);

  if (!addKmlPointBody(data_name, data_str, visibility)) return false;
  return true;
}

bool KmlGenerator::outputKml()
{
  kml_file_ofs_ << header_ << body_ <<  footer_ << std::endl;

  return true;
}

void KmlGenerator::setIntervalType(const IntervalType it)
{
  interval_type_ = it;
}

void KmlGenerator::setTimeInterval(const double time_interval)
{
  time_interval_ = time_interval;
}

void KmlGenerator::setPointInterval(const double point_interval)
{
  point_interval_ = point_interval;
}

void KmlGenerator::setLineInterval(const double line_interval)
{
  line_interval_ = line_interval;
}

KmlGenerator::IntervalType KmlGenerator::getIntervalType()
{
  return interval_type_;
}

double KmlGenerator::getTimeInterval()
{
  return time_interval_;
}

double KmlGenerator::getPointInterval()
{
  return point_interval_;
}

double KmlGenerator::getLineInterval()
{
  return line_interval_;
}

std::string KmlGenerator::getColorCode()
{
  std::string color_code;
  switch (color_type_) {
    case ColorType::RED:
      color_code = "ff0000ff";
      break;
    case ColorType::GREEN:
      color_code = "ff00ff00";
      break;
    case ColorType::BLUE:
      color_code = "ffff0000";
      break;
    case ColorType::PURPLE:
      color_code = "ffff55aa";
      break;
    case ColorType::CYAN:
      color_code = "ffffff00";
      break;
    case ColorType::MAGENTA:
      color_code = "ff7700ff";
      break;
    case ColorType::ORANGE:
      color_code = "ff00aaff";
      break;
    case ColorType::WHITE:
      color_code = "ffffffff";
      break;
  }
  return color_code;
}