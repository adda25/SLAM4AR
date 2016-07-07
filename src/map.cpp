#include "map.hpp"

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
int inline 
get_intersection(float fDst1, float fDst2, cv::Point3f P1, cv::Point3f P2, cv::Point3f &Hit) 
{
  if ((fDst1 * fDst2) >= 0.0f) return 0;
  if (fDst1 == fDst2) return 0; 
  Hit = P1 + (P2-P1) * (-fDst1 / (fDst2 - fDst1));
  return 1;
}

int inline 
in_box(cv::Point3f Hit, cv::Point3f B1, cv::Point3f B2, const int Axis) 
{
  if ( Axis==1 && Hit.z > B1.z && Hit.z < B2.z && Hit.y > B1.y && Hit.y < B2.y) return 1;
  if ( Axis==2 && Hit.z > B1.z && Hit.z < B2.z && Hit.x > B1.x && Hit.x < B2.x) return 1;
  if ( Axis==3 && Hit.x > B1.x && Hit.x < B2.x && Hit.y > B1.y && Hit.y < B2.y) return 1;
  return 0;
}

// returns true if line (L1, L2) intersects with the box (B1, B2)
// returns intersection point in Hit
int 
check_line_box(cv::Point3f B1, cv::Point3f B2, cv::Point3f L1, cv::Point3f L2, cv::Point3f &Hit)
{
  if (L2.x < B1.x && L1.x < B1.x) return false;
  if (L2.x > B2.x && L1.x > B2.x) return false;
  if (L2.y < B1.y && L1.y < B1.y) return false;
  if (L2.y > B2.y && L1.y > B2.y) return false;
  if (L2.z < B1.z && L1.z < B1.z) return false;
  if (L2.z > B2.z && L1.z > B2.z) return false;
  if (L1.x > B1.x && L1.x < B2.x &&
      L1.y > B1.y && L1.y < B2.y &&
      L1.z > B1.z && L1.z < B2.z) {
    Hit = L1; 
    return true;
  }
  if ( (get_intersection( L1.x-B1.x, L2.x-B1.x, L1, L2, Hit) && in_box( Hit, B1, B2, 1 ))
    || (get_intersection( L1.y-B1.y, L2.y-B1.y, L1, L2, Hit) && in_box( Hit, B1, B2, 2 )) 
    || (get_intersection( L1.z-B1.z, L2.z-B1.z, L1, L2, Hit) && in_box( Hit, B1, B2, 3 )) 
    || (get_intersection( L1.x-B2.x, L2.x-B2.x, L1, L2, Hit) && in_box( Hit, B1, B2, 1 )) 
    || (get_intersection( L1.y-B2.y, L2.y-B2.y, L1, L2, Hit) && in_box( Hit, B1, B2, 2 )) 
    || (get_intersection( L1.z-B2.z, L2.z-B2.z, L1, L2, Hit) && in_box( Hit, B1, B2, 3 ))) {
  	return true;
  }
  return false;
}
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////