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


void KmlGenerator::xyz2enu(double ecef_pos[3], double ecef_base_pos[3], double enu_pos[3])
{
  double llh_base_pos[3];
  ecef2llh(ecef_base_pos,llh_base_pos);
  enu_pos[0] = ((-(sin(llh_base_pos[1])) * (ecef_pos[0] - ecef_base_pos[0])) +
    ((cos(llh_base_pos[1])) * (ecef_pos[1] - ecef_base_pos[1])) + (0 * (ecef_pos[2] - ecef_base_pos[2])));
  enu_pos[1] = ((-(sin(llh_base_pos[0])) * (cos(llh_base_pos[1])) * (ecef_pos[0] - ecef_base_pos[0])) +
    (-(sin(llh_base_pos[0])) * (sin(llh_base_pos[1])) * (ecef_pos[1] - ecef_base_pos[1])) +
    ((cos(llh_base_pos[0])) * (ecef_pos[2] - ecef_base_pos[2])));
  enu_pos[2] = (((cos(llh_base_pos[0])) * (cos(llh_base_pos[1])) * (ecef_pos[0] - ecef_base_pos[0])) +
    ((cos(llh_base_pos[0])) * (sin(llh_base_pos[1])) * (ecef_pos[1] - ecef_base_pos[1])) +
    ((sin(llh_base_pos[0])) * (ecef_pos[2] - ecef_base_pos[2])));
}

void KmlGenerator::ecef2llh(double ecef_pos[3], double llh_pos[3])
{
  double semi_major_axis = 6378137.0000;
  double semi_minor_axis = 6356752.3142;
  double a1 = sqrt(1 - pow(semi_minor_axis / semi_major_axis, 2.0));
  double a2 = sqrt((ecef_pos[0] * ecef_pos[0]) + (ecef_pos[1] * ecef_pos[1]));
  double a3 = 54 * (semi_minor_axis * semi_minor_axis) * (ecef_pos[2] * ecef_pos[2]);
  double a4 = (a2 * a2) + (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2]) - (a1 * a1) * (semi_major_axis * semi_major_axis - semi_minor_axis * semi_minor_axis);
  double a5 = ((a1 * a1) * (a1 * a1) * a3 * (a2 * a2)) / (a4 * a4 * a4);
  double a6 = pow((1 + a5 + sqrt(a5 * a5 + 2 * a5)), 1.0 / 3.0);
  double a7 = a3 / (3 * pow((a6 + 1 / a6 + 1), 2.0) * a4 * a4);
  double a8 = sqrt(1 + 2 * (a1 * a1) * (a1 * a1) * a7);
  double a9 = -(a7 * (a1 * a1) * a2) / (1 + a8) + sqrt((semi_major_axis * semi_major_axis / 2) *
    (1 + 1 / a8) - (a7 * (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2])) / (a8 * (1 + a8)) - a7 * (a2 * a2) / 2);
  double a10 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (ecef_pos[2] * ecef_pos[2]));
  double a11 = sqrt((pow((a2 - (a1 * a1) * a9), 2.0)) + (1 - (a1 * a1)) * (ecef_pos[2] * ecef_pos[2]));
  double a12 = ((semi_minor_axis * semi_minor_axis) * ecef_pos[2]) / (semi_major_axis * a11);
  llh_pos[0] = atan((ecef_pos[2] + (a1 * (semi_major_axis / semi_minor_axis)) * (a1 * (semi_major_axis / semi_minor_axis)) * a12) / a2);
  llh_pos[1] = 0;

  if (ecef_pos[0] >= 0)
  {
    llh_pos[1] = (atan(ecef_pos[1] / ecef_pos[0]));
  }
  else
  {
    if (ecef_pos[0] < 0 && ecef_pos[1] >= 0)
    {
      llh_pos[1] = M_PI + (atan(ecef_pos[1] / ecef_pos[0]));
    }
    else
    {
      llh_pos[1] = (atan(ecef_pos[1] / ecef_pos[0])) - M_PI;
    }
  }

  llh_pos[2] = a10 * (1 - (semi_minor_axis * semi_minor_axis) / (semi_major_axis * a11));
}