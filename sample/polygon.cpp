#include <iostream>
#include <vector>
#include <kml_generator/kml_generator.hpp>

int main(int argc, char** argv)
{
  std::string kml_file = "test.kml";

  // Dummy data
  kml_utils::Point point1, point2, point3, point4;
  kml_utils::OtherInfo point1_other_info, point2_other_info, point3_other_info, point4_other_info;

  point1.latitude = 36.06105473611077;
  point1.longitude = 139.71861182416725;
  point1_other_info.name = "name";
  point1_other_info.value_str = "point1";
  point1.other_info_vector.push_back(point1_other_info);

  point2.latitude = 36.06107902781948;
  point2.longitude = 139.74767723631132;
  point2_other_info.name = "name";
  point2_other_info.value_str = "point2";
  point2.other_info_vector.push_back(point2_other_info);

  point3.latitude = 36.06952489910212;
  point3.longitude = 139.747668080499;
  point3_other_info.name = "name";
  point3_other_info.value_str = "point3";
  point3.other_info_vector.push_back(point3_other_info);

  point4.latitude = 36.06950059991341;
  point4.longitude = 139.71859956155666;
  point4_other_info.name = "name";
  point4_other_info.value_str = "point4";
  point4.other_info_vector.push_back(point4_other_info);

  kml_utils::Polygon polygon;
  polygon.header.name = "test.pcd";
  polygon.vertices.push_back(point1);
  polygon.vertices.push_back(point2);
  polygon.vertices.push_back(point3);
  polygon.vertices.push_back(point4);

  kml_utils::Point centroid;
  for (const auto& point : polygon.vertices)
  {
    centroid.latitude += point.latitude;
    centroid.longitude += point.longitude;
    centroid.altitude += point.altitude;
  }
  centroid.latitude /= polygon.vertices.size();
  centroid.longitude /= polygon.vertices.size();
  centroid.altitude /= polygon.vertices.size();
  centroid.other_info_vector = point1.other_info_vector;

  KmlGenerator kml_generator(kml_file);
  std::string polygon_style_id = "polygonStyle";
  std::string polygon_line_color = "bf00ffff";
  std::string polygon_line_width = "3f00ff00";
  kml_generator.addPolygonStyle(polygon_style_id, polygon_line_color, polygon_line_width);

  std::string label_style_id = "labelStyle";
  std::string label_color = "";
  std::string label_scale = "1.0";
  kml_generator.addLabelStyle(label_style_id, label_color, label_scale);

  kml_utils::Header boundary_folder;
  boundary_folder.name = "Boundaries";
  kml_generator.addKmlFolderBegin(boundary_folder);
  kml_generator.addPolygonKML(polygon, polygon_style_id);
  kml_generator.addKmlFolderEnd();

  kml_utils::Header file_name_folder;
  file_name_folder.name = "File names";
  kml_generator.addKmlFolderBegin(file_name_folder);
  kml_generator.addLabelKML(centroid, label_style_id);
  kml_generator.addKmlFolderEnd();

  kml_generator.outputKml();

  return 0;
}
