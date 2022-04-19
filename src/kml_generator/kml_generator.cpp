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

bool KmlGenerator::addKmlLineHeader(std::string data_name)
{
  std::string tmp_header;
  std::string color_str = color_list[(data_count_-1)%8];
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

bool KmlGenerator::addKmlLineBody(std::string data_name, std::string data_str)
{
  std::string tmp_body;
  tmp_body =
      "\t<Placemark>\n"
      "\t<name> " + data_name + " </name>\n"
      "\t<visibility>1</visibility>\n"
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

bool KmlGenerator::addKmlPointBody(std::string data_name, std::string data_str)
{
  std::string color_str = color_list[(data_count_-1)%8];
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
          "\t<name>CAR Trajectory</name>\n"
          "\t<visibility>1</visibility>\n"
          "\t<open>0</open>"
          ;
  body_ += "\t<Folder id=\"ID21\">\n"
    "\t<name> " + data_name + " </name>\n"
    "\t<visibility>0</visibility>\n";
  body_ += data_str;
  body_ += "\t</Folder>\n";
  body_ += "\t</Folder>\n";

  return true;
}

std::string KmlGenerator::NavSatFixMsgVector2LineStr(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector)
{
  std::stringstream data_ss;
  std::size_t data_length = std::distance(fix_msg_vector.begin(), fix_msg_vector.end());

  double time_last = 0;
  for (int i=0; i<data_length; i++)
  {
    if(fix_msg_vector[i].header.stamp.toSec() != time_last
      && fix_msg_vector[i].header.stamp.toSec() - time_last > 0.2
      && fix_msg_vector[i].longitude != 0 && fix_msg_vector[i].latitude != 0)
    {
      data_ss << std::setprecision(std::numeric_limits<double>::max_digits10) 
        << fix_msg_vector[i].longitude << ","
        << fix_msg_vector[i].latitude << ","
        << fix_msg_vector[i].altitude
        << std::endl;
      time_last = fix_msg_vector[i].header.stamp.toSec();
    }
  }

  return data_ss.str();
}

std::string KmlGenerator::NavSatFixMsg2PointStr(const sensor_msgs::NavSatFix fix_msg, std::string data_name)
{
  std::string data;
  data =
  "\t<Placemark>\n\
                \t\t<name>GNSS</name>\n\
                \t\t<Snippest></Snippest>\n\
                <description><![CDATA[<B>GNSS Status</B><BR><BR>\n\
                \t\t\t<TABLE border=\"1\" width=\"100%\" Align=\"center\">\n\
                \t\t\t\t<TR ALIGN=RIGHT>\n\
                \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Sequence Number: </TD><TD>" + std::to_string(fix_msg.header.seq) + "</TD></TR>\n"
                +
                "\t\t\t\t\t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>Timestamp: </TD><TD>"
            + std::to_string(fix_msg.header.stamp.toSec()) + "</TD></TR>\n\
                 \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>lat: </TD><TD>"
            + std::to_string(fix_msg.latitude) + "</TD></TR>\n\
                \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>lon: </TD><TD>"
            + std::to_string(fix_msg.longitude) + "</TD></TR>\n\
                \t\t\t\t<TR ALIGN=RIGHT><TD ALIGN=LEFT>height: </TD><TD>"
            + std::to_string(fix_msg.altitude) + "</TD></TR>\n\
                \t\t\t</TABLE>\n\
                ]]></description>\n\
                \t\t<styleUrl>"
            + "#" + data_name +
            + "</styleUrl>\n\
                \t\t<Point>\n\
                \t\t\t<coordinates>"
            + std::to_string(fix_msg.longitude) + ","
            + std::to_string(fix_msg.latitude) + ","
            + std::to_string(fix_msg.altitude) + ","
            + "</coordinates>\n\
                \t\t</Point>\n\
                \t</Placemark>";
  return data;
}

std::string KmlGenerator::NavSatFixMsgVector2PointStr(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, std::string data_name)
{
  std::string data;
  std::size_t data_length = std::distance(fix_msg_vector.begin(), fix_msg_vector.end());

  double time_last = 0;
  for (int i=0; i<data_length; i++)
  {
    if(fix_msg_vector[i].header.stamp.toSec() != time_last
      && fix_msg_vector[i].header.stamp.toSec() - time_last > 0.2
      && fix_msg_vector[i].longitude != 0 && fix_msg_vector[i].latitude != 0)
    {
      data += NavSatFixMsg2PointStr(fix_msg_vector[i], data_name);
    }
  }

  return data;
}

bool KmlGenerator::addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector)
{
  std::string data_str = NavSatFixMsgVector2LineStr(fix_msg_vector);
  std::string data_name;

  data_count_++;
  data_name = "DATANUM_" + std::to_string(data_count_);

  if (!addKmlLineHeader(data_name)) return false;
  if (!addKmlLineBody(data_name,data_str)) return false;
  return true;
}

bool KmlGenerator::addNavSatFixMsgVectorLine(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector, std::string data_name)
{
  std::string data_str = NavSatFixMsgVector2LineStr(fix_msg_vector);

  data_count_++;
  if (!addKmlLineHeader(data_name)) return false;
  if (!addKmlLineBody(data_name, data_str)) return false;
  return true;
}

bool KmlGenerator::addNavSatFixMsgVectorPoint(const std::vector<sensor_msgs::NavSatFix>& fix_msg_vector)
{

  std::string data_name;

  data_count_++;
  data_name = "DATANUM_" + std::to_string(data_count_);

  std::string data_str = NavSatFixMsgVector2PointStr(fix_msg_vector, data_name);

  if (!addKmlPointBody(data_name, data_str)) return false;
  return true;
}


bool KmlGenerator::outputKml()
{
  kml_file_ofs_ << header_ << body_ <<  footer_ << std::endl;

  return true;
}