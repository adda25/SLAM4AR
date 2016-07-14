#include "map.hpp"

// "Private routines"
std::vector<cv::Point3f>  _map__grid(const Map &map);
std::vector<cv::Point3f>  _line(cv::Point3f min_coords, cv::Point3f max_coords);
bool _is_coord_inside_box(cv::Point3f coord, const int box_bounds[6]);
int _map__find_nearest_sector_for_point(const Map &map, cv::Point3f coord);
MapSector _map__create_sector(const int reference_sector_bounds[6], cv::Point3f coord);
cv::Mat _cam_point_from_pixel(cv::Point3f point); 
cv::Point3f _box_center_from_bounds(const int bounds[6]);

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

Map 
map__create(uint sectors[3], uint sector_size[3]) {
  Map map;
  int total_half_side_lenght_x = (sectors[0] * sector_size[0]) * 0.5;
  int total_half_side_lenght_y = (sectors[1] * sector_size[1]) * 0.5;
  int total_half_side_lenght_z = (sectors[2] * sector_size[2]) * 0.5;
  for (int x = 0; x < sectors[0]; x++) {
    for (int y = 0; y < sectors[1]; y++) {
      for (int z = 0; z < sectors[2]; z++) {
        int xc_pos_m = -total_half_side_lenght_x + sector_size[0] * x;
        int yc_pos_m = -total_half_side_lenght_y + sector_size[1] * y;
        int zc_pos_m = -total_half_side_lenght_z + sector_size[2] * z;
        int xc_pos_M = -total_half_side_lenght_x + sector_size[0] * x + sector_size[0]; 
        int yc_pos_M = -total_half_side_lenght_y + sector_size[1] * y + sector_size[1];
        int zc_pos_M = -total_half_side_lenght_z + sector_size[2] * z + sector_size[2];
        MapSector sector = MapSector{xc_pos_m, yc_pos_m, zc_pos_m, xc_pos_M, yc_pos_M, zc_pos_M};
        map.push_back(sector);
      }
    }
  }
  return map;
}

void 
map__update(Map &map, 
            std::vector<MapPoint> points, 
            const cv::Mat pose, 
            const cv::Mat &frame) 
{
  for (auto &p : points) {
    // Get right sector
    int sector_index = map__sector_for_coords(map, p.coords_3D);
    if (sector_index == -1) {
      sector_index = _map__find_nearest_sector_for_point(map, p.coords_3D);
      MapSector new_sector = _map__create_sector(map[sector_index].sector_bounds, p.coords_3D);
      new_sector.sector_points.push_back(p);
      new_sector.sector_poses.push_back(pose);
      new_sector.sector_frames.push_back(frame);
      map.push_back(new_sector);
      continue;
    }
    // Update sector
    map[sector_index].sector_points.push_back(p);
    map[sector_index].sector_poses.push_back(pose);
    map[sector_index].sector_frames.push_back(frame);
  }
}

void 
map__remove_empty_sectors(Map &map) 
{
  for (size_t i = 0; i < map.size(); i++) {
    if (map[i].sector_points.size() == 0) {
      map.erase(map.begin() + i);
    }
  }
}

int 
map__sector_for_coords(Map &map, cv::Point3f coords_3D) 
{
  for (int i = 0; i < map.size(); i++) {
    if (_is_coord_inside_box(coords_3D, map[i].sector_bounds)) {
      return i;
    }
  } 
  return -1;
}

/* Returns the partial map in the
camera frustum */
Map 
map__sectors_in_view(const Map &map, cv::Mat camera_pose) 
{
  cv::Mat estimated_camera_pose;
  camera_pose.copyTo(estimated_camera_pose);
  estimated_camera_pose = estimated_camera_pose.inv();
  static bool first = true;
  Map partial_map;
  cv::Point3f box_e[2];
  cv::Point3f box_i[2];
  // Get 2 camera "box"
  cv::Mat be0 = estimated_camera_pose * _cam_point_from_pixel(cv::Point3f(-640, 360, 50));
  cv::Mat be1 = estimated_camera_pose * _cam_point_from_pixel(cv::Point3f(640,  -360, 55));
  //cv::Mat bi0 = estimated_camera_pose * _cam_point_from_pixel(cv::Point3f(-640, 360, 3.0));
  //cv::Mat bi1 = estimated_camera_pose * _cam_point_from_pixel(cv::Point3f(640,  -360, 5.0));
  box_e[0] = cv::Point3f(be0.at<double>(0, 0), be0.at<double>(1, 0), be0.at<double>(2, 0));
  box_e[1] = cv::Point3f(be1.at<double>(0, 0), be1.at<double>(1, 0), be1.at<double>(2, 0));
  //box_i[0] = cv::Point3f(bi0.at<double>(0, 0), bi0.at<double>(1, 0), bi0.at<double>(2, 0));
  //box_i[1] = cv::Point3f(bi1.at<double>(0, 0), bi1.at<double>(1, 0), bi1.at<double>(2, 0));
  // Get camera center
  cv::Point3f box_center;
  cv::Mat cam_c = estimated_camera_pose * _cam_point_from_pixel(cv::Point3f(640, 360, 0));
  cv::Point3f cam_center = cv::Point3f(cam_c.at<double>(0, 0), cam_c.at<double>(1, 0), cam_c.at<double>(2, 0));
  for (auto &sector : map) {
    if (sector.sector_points.size() == 0) {continue;}
    cv::Point3f hit;
    box_center = _box_center_from_bounds(sector.sector_bounds); // sector.sector_points[0].coords_3D_camera_sys;
    int inside_e = check_line_box(box_e[0], box_e[1], box_center, cam_center, hit);
    //int inside_i = check_line_box(box_i[0], box_i[1], box_center, cam_center, hit);
    if (inside_e == 1) {
        partial_map.push_back(sector);
    }
    //std::cout << inside_e << " " << " " << box_e[0] << " " << box_e[1] << " cam_center: " << cam_center << " box_center: " << box_center <<  std::endl;
  }
  /* if (first) { 
    std::ofstream ofs;
    ofs.open ("test_map.txt", std::ofstream::out | std::ofstream::app);
    std::vector<cv::Point3f> grid = _map__grid(map);
    for (auto &g : grid) {
      ofs << g.x << " " << g.y << " " << g.z << " " << 255 << " " << 0 << " " << 0 << "\n";   
    }
    for (auto &s : map) {
      for (auto &p : s.sector_points) {
        ofs << p.coords_3D.x << " " << p.coords_3D.y << " " << p.coords_3D.z << " " << 0 << " " << 255 << " " << 0 << "\n";
      }
    }
    ofs << cam_center.x << " " << cam_center.y << " " << cam_center.z << " " << 0 << " " << 0 << " " << 255 << "\n";
    ofs << box_e[0].x << " " << box_e[0].y << " " << box_e[0].z << " " << 0 << " " << 0 << " " << 255 << "\n";
    ofs << box_e[1].x << " " << box_e[1].y << " " << box_e[1].z << " " << 0 << " " << 0 << " " << 255 << "\n";
    for (auto &sector : map) {
      box_center = _box_center_from_bounds(sector.sector_bounds);
      ofs << box_center.x << " " << box_center.y << " " << box_center.z << " " << 0 << " " << 255 << " " << 255 << "\n";
    }

    ofs.close();
    first = false;
  } else {
    std::ofstream ofs;
    ofs.open ("test_map.txt", std::ofstream::out | std::ofstream::app);
    ofs << cam_center.x << " " << cam_center.y << " " << cam_center.z << " " << 0 << " " << 0 << " " << 255 << "\n";
    ofs << box_e[0].x << " " << box_e[0].y << " " << box_e[0].z << " " << 0 << " " << 0 << " " << 255 << "\n";
    ofs << box_e[1].x << " " << box_e[1].y << " " << box_e[1].z << " " << 0 << " " << 0 << " " << 255 << "\n";
    for (auto &sector : map) {
      box_center = _box_center_from_bounds(sector.sector_bounds);
      ofs << box_center.x << " " << box_center.y << " " << box_center.z << " " << 0 << " " << 255 << " " << 255 << "\n";
    }
    ofs.close();
  }*/
  std::cout << "partial_map: " << partial_map.size() << std::endl;
  if (partial_map.size() == 0) {
    return map;
  }
  return partial_map;
}

void 
map__draw(cv::Mat &frame_to_draw, const Map &map_to_draw, bool draw_grid) {
  
}

void 
map__write(std::string filename, const Map &map_to_write, bool write_grid) {
  std::ofstream ofs;
  ofs.open (filename, std::ofstream::out | std::ofstream::app);
  if (write_grid) {
    std::vector<cv::Point3f> grid = _map__grid(map_to_write);
    for (auto &g : grid) {
      ofs << g.x << " " << g.y << " " << g.z << " " << 255 << " " << 0 << " " << 0 << "\n";   
    }
  }
  for (auto &s : map_to_write) {
    for (auto &p : s.sector_points) {
        ofs << p.coords_3D.x << " " << p.coords_3D.y << " " << p.coords_3D.z << " " << 0 << " " << 255 << " " << 0 << "\n";
    }
  }
  ofs.close();
}

void 
map__merge_keypoints_and_descriptors(const Map &map, 
                                     std::vector<cv::KeyPoint> &keypoints_out, 
                                     cv::Mat &descriptors_out)
{
  for (auto &m : map) { 
    for (auto &s : m.sector_points) {
      descriptors_out.push_back(s.descriptor); 
      keypoints_out.push_back(s.keypoint); 
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<cv::Point3f> 
_map__grid(const Map &map) 
{
  std::vector<cv::Point3f> grid;
  for (auto &s : map) {
    // X
    cv::Point3f psx1 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[1], s.sector_bounds[2]);
    cv::Point3f psx2 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[1], s.sector_bounds[2]);
    std::vector<cv::Point3f> lx1 = _line(psx1, psx2);
    grid.insert(grid.end(), lx1.begin(), lx1.end());

    cv::Point3f psx3 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[4], s.sector_bounds[2]);
    cv::Point3f psx4 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[4], s.sector_bounds[2]);
    std::vector<cv::Point3f> lx2 = _line(psx3, psx4);
    grid.insert(grid.end(), lx2.begin(), lx2.end());

    cv::Point3f psx5 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[1], s.sector_bounds[5]);
    cv::Point3f psx6 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[1], s.sector_bounds[5]);
    std::vector<cv::Point3f> lx3 = _line(psx5, psx6);
    grid.insert(grid.end(), lx3.begin(), lx3.end());

    cv::Point3f psx7 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[4], s.sector_bounds[5]);
    cv::Point3f psx8 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[4], s.sector_bounds[5]);
    std::vector<cv::Point3f> lx4 = _line(psx7, psx8);
    grid.insert(grid.end(), lx4.begin(), lx4.end());

    // Y
    cv::Point3f psy1 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[1], s.sector_bounds[2]);
    cv::Point3f psy2 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[4], s.sector_bounds[2]);
    std::vector<cv::Point3f> ly1 = _line(psy1, psy2);
    grid.insert(grid.end(), ly1.begin(), ly1.end());

    cv::Point3f psy3 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[1], s.sector_bounds[5]);
    cv::Point3f psy4 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[4], s.sector_bounds[5]);
    std::vector<cv::Point3f> ly2 = _line(psy3, psy4);
    grid.insert(grid.end(), ly2.begin(), ly2.end());

    cv::Point3f psy5 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[1], s.sector_bounds[2]);
    cv::Point3f psy6 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[4], s.sector_bounds[2]);
    std::vector<cv::Point3f> ly3 = _line(psy5, psy6);
    grid.insert(grid.end(), ly3.begin(), ly3.end());

    cv::Point3f psy7 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[1], s.sector_bounds[5]);
    cv::Point3f psy8 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[4], s.sector_bounds[5]);
    std::vector<cv::Point3f> ly4 = _line(psy7, psy8);
    grid.insert(grid.end(), ly4.begin(), ly4.end());

    // Y
    cv::Point3f psz1 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[1], s.sector_bounds[2]);
    cv::Point3f psz2 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[1], s.sector_bounds[5]);
    std::vector<cv::Point3f> lz1 = _line(psz1, psz2);
    grid.insert(grid.end(), lz1.begin(), lz1.end());

    cv::Point3f psz3 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[4], s.sector_bounds[2]);
    cv::Point3f psz4 = cv::Point3f(s.sector_bounds[0], s.sector_bounds[4], s.sector_bounds[5]);
    std::vector<cv::Point3f> lz2 = _line(psz3, psz4);
    grid.insert(grid.end(), lz2.begin(), lz2.end());

    cv::Point3f psz5 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[1], s.sector_bounds[2]);
    cv::Point3f psz6 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[1], s.sector_bounds[5]);
    std::vector<cv::Point3f> lz3 = _line(psz5, psz6);
    grid.insert(grid.end(), lz3.begin(), lz3.end());

    cv::Point3f psz7 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[4], s.sector_bounds[2]);
    cv::Point3f psz8 = cv::Point3f(s.sector_bounds[3], s.sector_bounds[4], s.sector_bounds[5]);
    std::vector<cv::Point3f> lz4 = _line(psz7, psz8);
    grid.insert(grid.end(), lz4.begin(), lz4.end());
  }
  return grid;
}

std::vector<cv::Point3f>  
_line(cv::Point3f min_coords, cv::Point3f max_coords) 
{
  std::vector<cv::Point3f>  line;
  const int dt = 10;
  line.reserve(dt);
  float x_steps = fabs(min_coords.x - max_coords.x) / dt;
  float y_steps = fabs(min_coords.y - max_coords.y) / dt;
  float z_steps = fabs(min_coords.z - max_coords.z) / dt;
  for (int i = 0; i < dt; i++) {
    cv::Point3f p = cv::Point3f(min_coords.x + x_steps * i, 
                                min_coords.y + y_steps * i,
                                min_coords.z + z_steps * i);
    line.push_back(p);
  }
  return line;
}

bool 
_is_coord_inside_box(cv::Point3f coord, const int box_bounds[6]) 
{
  return (coord.x < box_bounds[3] && 
          coord.x > box_bounds[0] &&
          coord.y < box_bounds[4] &&
          coord.y > box_bounds[1] &&
          coord.z < box_bounds[5] &&
          coord.z > box_bounds[2]);
}

int 
_map__find_nearest_sector_for_point(const Map &map, cv::Point3f coord) 
{
  float min_dist = 1000000.0f;
  int min_index  = 0;
  for (int i = 0; i < map.size(); i++) {
    float d_x = float(map[i].sector_bounds[3] + map[i].sector_bounds[0]) * 0.5f - coord.x;
    float d_y = float(map[i].sector_bounds[4] + map[i].sector_bounds[1]) * 0.5f - coord.y;
    float d_z = float(map[i].sector_bounds[5] + map[i].sector_bounds[2]) * 0.5f - coord.z;
    float distance = sqrt(pow(d_x, 2) + pow(d_y, 2) + pow(d_z, 2));
    if (distance < min_dist) {
        min_dist = distance;
        min_index = i;
    }
  }
  return min_index;
}

MapSector
_map__create_sector(const int reference_sector_bounds[6], cv::Point3f coord) 
{
  float s_x = float(reference_sector_bounds[3] - reference_sector_bounds[0]);
  float s_y = float(reference_sector_bounds[4] - reference_sector_bounds[1]);
  float s_z = float(reference_sector_bounds[5] - reference_sector_bounds[2]);
  float c_x = float(reference_sector_bounds[3] + reference_sector_bounds[0]) * 0.5f;
  float c_y = float(reference_sector_bounds[4] + reference_sector_bounds[1]) * 0.5f;
  float c_z = float(reference_sector_bounds[5] + reference_sector_bounds[2]) * 0.5f;
  int x_dir = coord.x > c_x ? 1 : -1;
  int y_dir = coord.y > c_y ? 1 : -1;
  int z_dir = coord.z > c_z ? 1 : -1;
  bool outside = true;
  int counter[3] = {0, 0, 0};
  while (outside) {
    int matchs = 0;
    if (coord.x < c_x + s_x * 0.5 + s_x * counter[0] * x_dir && 
        coord.x > c_x - s_x * 0.5 + s_x * counter[0] * x_dir) {
      matchs++;
    } else {
        counter[0]++;
    }
    if (coord.y < c_y + s_y * 0.5 + s_y * counter[1] * y_dir && 
        coord.y > c_y - s_y * 0.5 + s_y * counter[1] * y_dir) {
      matchs++;
    } else {
        counter[1]++;
    }
    if (coord.z < c_z + s_z * 0.5 + s_z * counter[2] * z_dir && 
        coord.z > c_z - s_z * 0.5 + s_z * counter[2] * z_dir) {
      matchs++;
    } else {
        counter[2]++;
    }
    if (matchs == 3) {
        outside = false;
    }
  }
  return MapSector{(int)(c_x - s_x * 0.5 + s_x * counter[0] * x_dir), 
                   (int)(c_y - s_y * 0.5 + s_y * counter[1] * y_dir),
                   (int)(c_z - s_z * 0.5 + s_z * counter[2] * z_dir),
                   (int)(c_x + s_x * 0.5 + s_x * counter[0] * x_dir),
                   (int)(c_y + s_y * 0.5 + s_y * counter[1] * y_dir),
                   (int)(c_z + s_z * 0.5 + s_z * counter[2] * z_dir)};
}

cv::Mat
_cam_point_from_pixel(cv::Point3f point) 
{
  cv::Mat result(4, 1, CV_64FC1);
  result.at<double>(0,0) = (point.x - 640) * point.z / 1152;
  result.at<double>(1,0) = (point.y - 360) * point.z / 1152;
  result.at<double>(2,0) = point.z;
  result.at<double>(3,0) = 1;
  return result;
}

cv::Point3f 
_box_center_from_bounds(const int bounds[6]) 
{
  float c_x = float(bounds[3] + bounds[0]) * 0.5f;
  float c_y = float(bounds[4] + bounds[1]) * 0.5f;
  float c_z = float(bounds[5] + bounds[2]) * 0.5f;
  return cv::Point3f(c_x, c_y, c_z);
}



