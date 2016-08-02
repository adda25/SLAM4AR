/*
 __  __             
|  \/  | __ _ _ __  
| |\/| |/ _` | '_ \ 
| |  | | (_| | |_) |
|_|  |_|\__,_| .__/ 
             |_|    
*/

#include "map.hpp"

// "Private routines"
std::vector<cv::Point3f>  _map__grid(const Map &map);
std::vector<cv::Point3f>  _line(cv::Point3f min_coords, cv::Point3f max_coords);
bool _is_coord_inside_box(cv::Point3f coord, const int box_bounds[6]);
int _map__find_nearest_sector_for_point(const Map &map, cv::Point3f coord);
MapSector _map__create_sector(const int reference_sector_bounds[6], cv::Point3f coord);
cv::Mat _cam_point_from_pixel(cv::Point3f point); 
cv::Point3f _box_center_from_bounds(const int bounds[6]);
bool _map__similar_point_in_sector(MapPoint point, MapSector &sector);
bool _point_inside_pyramid(cv::Point3f py_center, 
                           cv::Point3f py_up_left, 
                           cv::Point3f py_up_right,
                           cv::Point3f py_down_right,
                           cv::Point3f py_down_left,
                           cv::Point3f point_to_check);
cv::Mat _vector_from_2_points(cv::Point3f p1, cv::Point3f p2);


std::vector<std::string> split(std::string data, std::string token);

int  
get_intersection(float fDst1, float fDst2, cv::Point3f P1, cv::Point3f P2, cv::Point3f &Hit) 
{
  if ((fDst1 * fDst2) >= 0.0f) return 0;
  if (fDst1 == fDst2) return 0; 
  Hit = P1 + (P2-P1) * (-fDst1 / (fDst2 - fDst1));
  return 1;
}

int  
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
    if (p.coords_3D.x != p.coords_3D.x) { continue; }
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
    if (_map__similar_point_in_sector(p, map[sector_index])) { continue; }
    map[sector_index].sector_points.push_back(p);
    map[sector_index].sector_poses.push_back(pose);
    map[sector_index].sector_frames.push_back(frame);
  }
}

std::vector<cv::DMatch> 
_map__match_features(cv::Mat descriptors_1, cv::Mat descriptors_2) 
{
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> tot_matches;
  size_t t_size = 0;
  cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
  matcher.knnMatch(descriptors_1, descriptors_2, matches, 1);
  for (auto &m : matches) { t_size += m.size(); }
  tot_matches.reserve(t_size);
  for (auto &m : matches) { tot_matches.insert(tot_matches.end(), m.begin(), m.end()); }
  return tot_matches;  
}

bool 
_map__similar_point_in_sector(MapPoint point, MapSector &sector)
{
  for (auto &p : sector.sector_points) {
    std::vector<cv::DMatch>  dm = _map__match_features(point.descriptor, p.descriptor);
    if (dm[0].distance < 0.1) {
        std::cout << "--> Refining " << p.coords_3D << " " << point.coords_3D << std::endl;
        p.coords_3D.x = (p.coords_3D.x + point.coords_3D.x) * 0.5;
        p.coords_3D.x = (p.coords_3D.y + point.coords_3D.y) * 0.5;
        p.coords_3D.x = (p.coords_3D.z + point.coords_3D.z) * 0.5;
        p.hits++;
        return true;
    }
  }
  return false;
}

void 
map__remove_empty_sectors(Map &map, uint min_size) 
{
  for (size_t i = 0; i < map.size(); i++) {
    if (map[i].sector_points.size() <= min_size) {
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
map__sectors_in_view(CameraSystem &camera, 
                     const Map &map, 
                     cv::Mat camera_pose) 
{
  cv::Mat estimated_camera_pose;
  camera_pose.copyTo(estimated_camera_pose);
  estimated_camera_pose = estimated_camera_pose.inv();
  //std::cout << estimated_camera_pose << std::endl;
  Map partial_map;
  cv::Point3f cam_center, up_left_pt, up_right_pt, down_right_pt, down_left_pt;
  cv::Point3f box_center;

  // Get camera center
  cv::Mat cam_c = estimated_camera_pose * camera.cam_point_from_pixel(cv::Point(camera.x_center, camera.y_center), 0);
  cam_center = cv::Point3f(cam_c.at<double>(0, 0), cam_c.at<double>(1, 0), cam_c.at<double>(2, 0));

  // Up-Left
  cv::Mat u_l_pt = estimated_camera_pose * camera.cam_point_from_pixel(cv::Point(0, 0), 0);
  up_left_pt = cv::Point3f(u_l_pt.at<double>(0, 0), u_l_pt.at<double>(1, 0), u_l_pt.at<double>(2, 0));

  // Up-Right
  cv::Mat u_r_pt = estimated_camera_pose * camera.cam_point_from_pixel(cv::Point(camera.cam_size.width, 0), 1000);
  up_right_pt = cv::Point3f(u_r_pt.at<double>(0, 0), u_r_pt.at<double>(1, 0), u_r_pt.at<double>(2, 0));

  // Down-Right
  cv::Mat d_r_pt = estimated_camera_pose * camera.cam_point_from_pixel(cv::Point(camera.cam_size.width, camera.cam_size.height), 1000);
  down_right_pt = cv::Point3f(d_r_pt.at<double>(0, 0), d_r_pt.at<double>(1, 0), d_r_pt.at<double>(2, 0));

  // Down-Left
  cv::Mat d_l_pt = estimated_camera_pose * camera.cam_point_from_pixel(cv::Point(0, camera.cam_size.height), 1000);
  down_left_pt = cv::Point3f(d_l_pt.at<double>(0, 0), d_l_pt.at<double>(1, 0), d_l_pt.at<double>(2, 0));

  //std::cout << up_left_pt << " " << up_right_pt << " " << down_right_pt << " " << down_left_pt << std::endl;

  for (auto &sector : map) {
    if (sector.sector_points.size() == 0) {continue;}
    cv::Point3f hit;
    box_center = _box_center_from_bounds(sector.sector_bounds); // sector.sector_points[0].coords_3D_camera_sys;
    if (_point_inside_pyramid(cam_center, up_left_pt, up_right_pt, down_right_pt, down_left_pt, box_center)) {
        partial_map.push_back(sector);
    }
  }
  //std::cout << "partial_map: " << partial_map.size() << std::endl;
  if (partial_map.size() == 0) {
    return map;
  }
  return partial_map;
}

void 
map__draw(cv::Mat &frame_to_draw, 
          const Map &map_to_draw, 
          cv::Mat tr_vec, 
          cv::Mat rot_vec, 
          const cv::Mat camera_matrix, 
          const cv::Mat camera_distorsion,
          bool draw_grid)
{
  std::vector<cv::Point2f> output_points;
  std::vector<MapPoint> me_map;
  std::vector<cv::Point2f> image;
  std::vector<cv::Point3f> object_points;
  map__merge(map_to_draw, me_map);
  for (auto &m : me_map) {
    object_points.push_back(m.coords_3D);
  }
  cv::projectPoints(object_points, rot_vec, 
                    tr_vec, camera_matrix, 
                    camera_distorsion, output_points); 
  for (auto &p : output_points) {
    cv::circle(frame_to_draw, p, 5, cv::Scalar(0,0,255,255));
  }
}

void 
map__write(std::string filename, const Map &map_to_write, bool write_grid) 
{
  std::ofstream ofs;
  ofs.open (filename, std::ofstream::trunc);
  if (write_grid) {
    std::vector<cv::Point3f> grid = _map__grid(map_to_write);
    for (auto &g : grid) {
      ofs << g.x << " " << g.y << " " << g.z << " " << 255 << " " << 0 << " " << 0 << "\n";   
    }
  }
  for (auto &s : map_to_write) {
    for (auto &p : s.sector_points) {
      ofs << p.coords_3D.x << " " << p.coords_3D.y << " " << p.coords_3D.z << " " << (int)p.pixel_colors.val[2] << " " << (int)p.pixel_colors.val[1] << " " << (int)p.pixel_colors.val[0] << "\n";
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

void 
map__merge(const Map &map, std::vector<MapPoint> &me_map)
{
  for (auto &m : map) { 
    for (auto &s : m.sector_points) {
      MapPoint mp = s;
      me_map.push_back(mp);
    }
  }
}

void 
map__image_points_for_pose(const Map &map, 
                           cv::Mat current_tr,
                           cv::Mat current_rot, 
                           cv::Mat camera_matrix,
                           cv::Mat camera_distorsion,
                           std::vector<MapPoint> &image_points_out)
{ 
  for (auto &m : map) {
    for (auto &p : m.sector_points) {
      std::vector<cv::Point2f> im_pt;
      std::vector<cv::Point3f> p_vec;
      MapPoint nmp = p;
      p_vec.push_back(p.coords_3D);
      cv::projectPoints(p_vec, current_rot, current_tr, 
                        camera_matrix, camera_distorsion, im_pt);
      nmp.point_pixel_pos = im_pt[0];
      image_points_out.push_back(nmp);
    }
  }  
}

void 
map__save_on_file(std::string filename, const Map &map_to_write)
{
  std::ofstream ofs;
  ofs.open (filename, std::ofstream::trunc);
  for (auto &m : map_to_write) {
    if (m.sector_points.size() == 0) { continue; }
    ofs << "#SECTOR\n";
    ofs << "#BOUNDS " << m.sector_bounds[0] 
        << " " << m.sector_bounds[1] 
        << " " << m.sector_bounds[2] 
        << " " << m.sector_bounds[3] 
        << " " << m.sector_bounds[4] 
        << " " << m.sector_bounds[5] << "\n";
    for (auto &p : m.sector_points) {
        ofs << "#MAP_POINT\n";
        ofs << "#MP-C3D " << p.coords_3D.x << " " << p.coords_3D.y << " " << p.coords_3D.z << "\n";
        ofs << "#MP-KP " << p.keypoint.pt.x << " " << p.keypoint.pt.y << "\n";
        ofs << "#MP-DS ";
        //std::cout << (uint)(uint8_t)p.descriptor.at<uchar>(0,3) << " " << (uint)(uint8_t)p.descriptor.at<uchar>(3,0) << std::endl;
        for (int i = 0; i < 32; i++) { ofs << (uint)(uint8_t)p.descriptor.at<uchar>(0,i) << " "; }
        ofs << "\n";
    }
    ofs << "\n";
  }
  ofs.close();
}

Map 
map__load_from_file(std::string filename)
{
  std::string line;  
  Map loaded_map;
  std::ifstream ifs;
  ifs.open (filename, std::ifstream::in);
  while (getline(ifs, line)) {
    std::vector<std::string> line_ary = split(line, " ");
    if (line_ary[0] == "#SECTOR") {
      loaded_map.push_back(MapSector());
    } else if (line_ary[0] == "#BOUNDS") {
      int bounds[6] = {stoi(line_ary[1]),
                       stoi(line_ary[2]),
                       stoi(line_ary[3]),
                       stoi(line_ary[4]),
                       stoi(line_ary[5]),
                       stoi(line_ary[6])}; 
      for (int i = 0; i < 6; i++) { loaded_map.back().sector_bounds[i] = bounds[i]; }
    } else if (line_ary[0] == "#MAP_POINT") {
      loaded_map.back().sector_points.push_back(MapPoint());
      loaded_map.back().sector_points.back().descriptor = cv::Mat(1, 32, CV_8U);
    } else if (line_ary[0] == "#MP-C3D") {
      loaded_map.back().sector_points.back().coords_3D = cv::Point3f(stof(line_ary[1]), 
                                                                     stof(line_ary[2]), 
                                                                     stof(line_ary[3]));
    } else if (line_ary[0] == "#MP-KP") {
      loaded_map.back().sector_points.back().keypoint.pt = cv::Point2f(stof(line_ary[1]), 
                                                                       stof(line_ary[2]));
    } else if (line_ary[0] == "#MP-DS") {
      for (int i = 0; i < line_ary.size() - 2; i++) {
        int value  = (int)stoi(line_ary[i+1]);
        loaded_map.back().sector_points.back().descriptor.at<uchar>(0,i) = value;
      }
    }
  }
  ifs.close();
  return loaded_map;
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

    // Z
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

bool 
_point_inside_pyramid(cv::Point3f py_center, 
                      cv::Point3f py_up_left, 
                      cv::Point3f py_up_right,
                      cv::Point3f py_down_right,
                      cv::Point3f py_down_left,
                      cv::Point3f point_to_check)
{
  // Right face check  
  cv::Mat up_right = _vector_from_2_points(py_center, py_up_right);
  cv::Mat dw_right = _vector_from_2_points(py_center, py_down_right);
  cv::Mat target_up_right = _vector_from_2_points(point_to_check, py_up_right);
  cv::Mat cross_up_right__dw__right = up_right.cross(dw_right);
  double dot_rf_t = target_up_right.dot(cross_up_right__dw__right);
  // Left face check  
  cv::Mat up_left = _vector_from_2_points(py_center, py_up_left);
  cv::Mat dw_left = _vector_from_2_points(py_center, py_down_left);
  cv::Mat target_up_left = _vector_from_2_points(point_to_check, py_up_left);
  cv::Mat cross_up_left__dw__left = up_left.cross(dw_left);
  double dot_lf_t = target_up_right.dot(cross_up_left__dw__left);
  // Up face check
  cv::Mat target_up = _vector_from_2_points(point_to_check, py_up_left);
  cv::Mat cross_up_right__up__left = up_right.cross(up_left);
  double dot_u_t = target_up.dot(cross_up_right__up__left);
  // Down face check
  cv::Mat target_dw_right = _vector_from_2_points(point_to_check, py_down_right);
  cv::Mat cross_dw_right__dw__left = dw_right.cross(dw_left);
  double dot_d_t = target_dw_right.dot(cross_dw_right__dw__left);
  if (dot_rf_t < 0 && 
      dot_lf_t < 0 &&
      dot_u_t  > 0 &&
      dot_d_t  < 0) {
        return true;
      }  
  return false;
}

cv::Mat 
_vector_from_2_points(cv::Point3f p1, cv::Point3f p2)
{
  cv::Mat v = cv::Mat (1, 3, CV_32F);
  float mod = 1;
  v.at<float>(0,0) = p2.x - p1.x;
  v.at<float>(0,1) = p2.y - p1.y;
  v.at<float>(0,2) = p2.z - p1.z;
  mod = sqrt(pow(v.at<float>(0,0), 2) + pow(v.at<float>(0,1), 2) + pow(v.at<float>(0,2), 2));
  v.at<float>(0,0) /= mod;
  v.at<float>(0,1) /= mod;
  v.at<float>(0,2) /= mod;
  return v;
}

/// INSERT IN AUX
std::vector<std::string> 
split(std::string data, std::string token)
{
  std::vector<std::string> output;
  size_t pos = std::string::npos;
  do {
    pos = data.find(token);
    output.push_back(data.substr(0, pos));
    if (std::string::npos != pos)
      data = data.substr(pos + token.size());
    } while (std::string::npos != pos);
  return output;
}


