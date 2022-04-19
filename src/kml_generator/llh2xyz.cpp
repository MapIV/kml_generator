#include <kml_generator/kml_generator.hpp>

// Ref:eagleye(BSD3 License)
// https://github.com/MapIV/eagleye/blob/main-ros1/eagleye_core/coordinate/src/llh2xyz.cpp
void KmlGenerator::llh2xyz(double llh_pos[3], double ecef_pos[3])
{
  double semi_major_axis = 6378137.0000;
  double semi_minor_axis = 6356752.3142;
  double a1 = sqrt (1-pow((semi_minor_axis/semi_major_axis), 2.0));
  double a2 = a1 * a1;

  double phi = llh_pos[0];
  double lam = llh_pos[1];
  double hei = llh_pos[2];

  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double cos_lam = cos(lam);
  double sin_lam = sin(lam);

  double tmp1 = 1 - a2;
  double tmp2 = sqrt(1 - a2*sin_phi*sin_phi);

  ecef_pos[0] = (semi_major_axis/tmp2 + hei)*cos_lam*cos_phi;
  ecef_pos[1] = (semi_major_axis/tmp2 + hei)*sin_lam*cos_phi;
  ecef_pos[2] = (semi_major_axis/tmp2*tmp1 + hei)*sin_phi;
}